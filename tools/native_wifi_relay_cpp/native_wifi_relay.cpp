#include <dds/dds.h>
#include <dds/ddsc/dds_public_qos.h>
#include <dds/ddsc/dds_public_status.h>
#include <signal.h>
#include <atomic>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <thread>
#include <string>

extern "C" {
#include "relay_types.h"
#include "unitree_relay.h"
}

static std::atomic<bool> running{true};
static bool probe_only = false;

static void stop_handler(int) { running.store(false, std::memory_order_relaxed); }

static std::string config(const char *interface, bool wifi) {
  std::string xml = "<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"";
  xml += interface;
  xml += "\" multicast=\"true\"/></Interfaces><AllowMulticast>true</AllowMulticast></General>";
  if (wifi) xml += "<Discovery><Peers><Peer Address=\"192.168.76.2\"/></Peers></Discovery>";
  xml += "</Domain></CycloneDDS>";
  return xml;
}

template <const dds_topic_descriptor_t *Desc>
static void relay_topic(dds_entity_t input_participant, dds_entity_t output_participant,
                        const char *input_name, const char *output_name, const char *label,
                        bool reliable = false) {
  dds_qos_t *reader_qos = dds_create_qos();
  dds_qset_reliability(reader_qos, reliable ? DDS_RELIABILITY_RELIABLE : DDS_RELIABILITY_BEST_EFFORT,
                       reliable ? DDS_MSECS(100) : 0);
  dds_qset_history(reader_qos, DDS_HISTORY_KEEP_LAST, reliable ? 4 : 1);
  dds_entity_t topic_in = dds_create_topic(input_participant, Desc, input_name, nullptr, nullptr);
  dds_entity_t reader = dds_create_reader(input_participant, topic_in, reader_qos, nullptr);
  dds_delete_qos(reader_qos);

  auto *writer_qos = dds_create_qos();
  dds_qset_reliability(writer_qos, reliable ? DDS_RELIABILITY_RELIABLE : DDS_RELIABILITY_BEST_EFFORT,
                       reliable ? DDS_MSECS(100) : 0);
  dds_qset_history(writer_qos, DDS_HISTORY_KEEP_LAST, reliable ? 4 : 1);
  dds_entity_t topic_out = dds_create_topic(output_participant, Desc, output_name, nullptr, nullptr);
  dds_entity_t writer = dds_create_writer(output_participant, topic_out, writer_qos, nullptr);
  dds_delete_qos(writer_qos);
  if (reader < 0 || writer < 0) {
    std::fprintf(stderr, "%s: input=%d reader=%d output=%d writer=%d\n",
                 label, topic_in, reader, topic_out, writer);
    return;
  }
  std::fprintf(stderr, "%s ready: %s -> %s (%s)\n", label, input_name, output_name, Desc->m_typename);

  void *samples[8]{};
  dds_sample_info_t infos[8]{};
  unsigned long long frames = 0, write_errors = 0;
  auto report_at = std::chrono::steady_clock::now() + std::chrono::seconds(5);
  while (running.load(std::memory_order_relaxed)) {
    dds_return_t n = dds_take(reader, samples, infos, sizeof(samples), 8);
    if (n < 0) {
      std::this_thread::sleep_for(std::chrono::microseconds(200));
      continue;
    }
    for (dds_return_t i = 0; i < n; ++i) {
      if (!infos[i].valid_data || !samples[i]) continue;
      const dds_return_t rc = dds_write(writer, samples[i]);
      if (rc >= 0) ++frames;
      else ++write_errors;
    }
    dds_return_loan(reader, reinterpret_cast<void **>(samples), n);
    if (std::chrono::steady_clock::now() >= report_at) {
      dds_subscription_matched_status_t matched{};
      dds_requested_incompatible_qos_status_t incompatible{};
      dds_get_subscription_matched_status(reader, &matched);
      dds_get_requested_incompatible_qos_status(reader, &incompatible);
      dds_publication_matched_status_t writer_matched{};
      dds_offered_incompatible_qos_status_t writer_incompatible{};
      if (writer > 0) {
        dds_get_publication_matched_status(writer, &writer_matched);
        dds_get_offered_incompatible_qos_status(writer, &writer_incompatible);
      }
      dds_inconsistent_topic_status_t inconsistent{};
      dds_get_inconsistent_topic_status(topic_in, &inconsistent);
      std::fprintf(stderr, "relay[%s] frames=%llu write_errors=%llu reader_matches=%u reader_incompatible=%u writer_matches=%u writer_incompatible=%u last_policy=%u inconsistent_topic=%u\n",
                   label, frames, write_errors, matched.current_count, incompatible.total_count,
                   writer_matched.current_count, writer_incompatible.total_count,
                   incompatible.last_policy_id, inconsistent.total_count);
      report_at = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    }
    if (n == 0) std::this_thread::sleep_for(std::chrono::microseconds(200));
  }
  dds_delete(reader);
  dds_delete(topic_in);
  if (writer > 0) dds_delete(writer);
  if (topic_out > 0) dds_delete(topic_out);
}

int main(int argc, char **argv) {
  signal(SIGINT, stop_handler);
  signal(SIGTERM, stop_handler);
  const bool source = argc > 1 && std::strcmp(argv[1], "--source") == 0;
  const bool wifi = argc > 1 && std::strcmp(argv[1], "--wifi") == 0;
  if (!source && !wifi) {
    std::fprintf(stderr, "Usage: native_wifi_relay_cpp --source|--wifi\n");
    return 2;
  }
  const char *external_interface = source ? "eth0" : "wlan0";
  auto external_config = config(external_interface, wifi);
  auto internal_config = config("lo", false);
  const dds_domainid_t external_domain_id = 0;
  const dds_domainid_t internal_domain_id = 1;
  dds_entity_t external_domain = dds_create_domain(external_domain_id, external_config.c_str());
  dds_entity_t internal_domain = dds_create_domain(internal_domain_id, internal_config.c_str());
  if (external_domain < 0 || internal_domain < 0) {
    std::fprintf(stderr, "create domains failed: external=%d internal=%d\n", external_domain, internal_domain);
    return 2;
  }
  dds_entity_t external = dds_create_participant(external_domain_id, nullptr, nullptr);
  dds_entity_t internal = dds_create_participant(internal_domain_id, nullptr, nullptr);
  if (external < 0 || internal < 0) {
    std::fprintf(stderr, "create participants failed: external=%d internal=%d\n", external, internal);
    return 2;
  }
  const char *cloud_external = "rt/utlidar/cloud";
  const char *imu_external = "rt/utlidar/imu";
  const char *sport_external = "rt/sportmodestate";
  const char *low_external = "rt/lowstate";
  const char *cloud_local = "rt/relay/internal/utlidar/cloud";
  const char *imu_local = "rt/relay/internal/utlidar/imu";
  const char *sport_local = "rt/relay/internal/sportmodestate";
  const char *low_local = "rt/relay/internal/lowstate";
  std::thread cloud(relay_topic<&sensor_msgs_msg_dds__PointCloud2__desc>,
                    source ? external : internal, source ? internal : external,
                    source ? cloud_external : cloud_local, source ? cloud_local : cloud_external, "cloud", false);
  std::thread imu(relay_topic<&sensor_msgs_msg_dds__Imu__desc>,
                  source ? external : internal, source ? internal : external,
                  source ? imu_external : imu_local, source ? imu_local : imu_external, "imu", false);
  std::thread sport(relay_topic<&unitree_go_msg_dds__SportModeState__desc>,
                    source ? external : internal, source ? internal : external,
                    source ? sport_external : sport_local, source ? sport_local : sport_external, "sportmodestate", false);
  std::thread low(relay_topic<&unitree_go_msg_dds__LowState__desc>,
                  source ? external : internal, source ? internal : external,
                  source ? low_external : low_local, source ? low_local : low_external, "lowstate", false);
  std::thread requests(relay_topic<&unitree_api_msg_dds__Request__desc>,
                       wifi ? external : internal, wifi ? internal : external,
                       wifi ? "rt/go2/wifi/sport/request" : "rt/relay/internal/sport/request",
                       wifi ? "rt/relay/internal/sport/request" : "rt/api/sport/request",
                       "sport_request", true);
  cloud.join();
  imu.join();
  sport.join();
  low.join();
  requests.join();
  dds_delete(external);
  dds_delete(internal);
  dds_delete(external_domain);
  dds_delete(internal_domain);
  return 0;
}
