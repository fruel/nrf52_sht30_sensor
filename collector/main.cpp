#include <iostream>
#include <atomic>
#include <sstream>
#include <csignal>
#include <errno.h>
#include <iomanip>
#include <unistd.h>
#include <chrono>
#include <sys/socket.h>
#include <unordered_map>
#include <curl/curl.h>

#include <bluetooth/bluetooth.h>
#include <bluetooth/hci.h>
#include <bluetooth/hci_lib.h>

#define S(x) #x
#define S_(x) S(x)
#define CHECK(call) { auto rc = (call); if(rc < 0) { std::cerr << strerror(errno) << " at " __FILE__ ":" S_(__LINE__); return -1;} }

#pragma pack(push)
#pragma pack(1)
struct SensorData {
    uint16_t msg_id;
    int32_t temperature;
    int32_t humidity;
    uint16_t battery_voltage;
    uint8_t battery_percentage;
};
#pragma pack(pop)

struct Node {
    std::string id;
    std::string name;
    uint16_t last_seen_msg_id;
    bool first_message;
};

static const std::string influx_url = "http://your.influxdb.host:8086";
static const std::string influx_db = "sensors";

// MAC address whitelist (+ additional hardcoded device information)
// TODO: add your beacons 
static std::unordered_map<std::string, Node> whitelist {
    {"00:00:00:00:00:00", {"1", "Kitchen", 0, true}}
};

static std::atomic<bool> run(true);

static void signal_handler(int)
{
    run = false;
}

static void send_to_influx(const Node& node, const SensorData& data, uint32_t loss)
{     
    CURL* curl = curl_easy_init();
    
    std::stringstream url;
    url << influx_url << "/write?db=" << influx_db;
    
    std::stringstream record;
    record << node.name << " ";
    record << "temp=" << (data.temperature / 1000.f) << ",";
    record << "humidity=" << (data.humidity / 1000.f) << ",";
    record << "batt_mv=" << data.battery_voltage << ",";
    record << "batt_percent=" << static_cast<int>(data.battery_percentage) << ",";
    record << "packet_loss=" << loss;

    curl_easy_setopt(curl, CURLOPT_URL, url.str().c_str());
    curl_easy_setopt(curl, CURLOPT_COPYPOSTFIELDS, record.str().c_str());
 
    CURLcode  res = curl_easy_perform(curl);

    if(res != CURLE_OK)
    {
        std::cerr << "error: curl " << curl_easy_strerror(res);
    }
 
    curl_easy_cleanup(curl);
}

int main(int argc, char** argv)
{
    curl_global_init(CURL_GLOBAL_ALL);
    std::signal(SIGINT, &signal_handler);
    std::signal(SIGKILL, &signal_handler);

    int dev_id = hci_get_route(nullptr);

    int device = hci_open_dev(dev_id);
    CHECK(device);    
    CHECK(hci_le_set_scan_parameters(device, 0, htobs(0x0010), htobs(0x0010), 0, 0, 1000));
    CHECK(hci_le_set_scan_enable(device, 1, 0, 1000));

    struct hci_filter nf;
    hci_filter_clear(&nf);
    hci_filter_set_ptype(HCI_EVENT_PKT, &nf);
    hci_filter_set_event(EVT_LE_META_EVENT, &nf);

    CHECK(setsockopt(device, SOL_HCI, HCI_FILTER, &nf, sizeof(nf)));
    
    struct sigaction sa = {};
    sa.sa_flags = SA_NOCLDSTOP;
    sigaction(SIGINT, &sa, NULL);

    uint8_t buf[HCI_MAX_EVENT_SIZE];
    while(run) 
    {
        int len = read(device, buf, sizeof(buf));

        if(len < 0)
        {
            if (errno == EAGAIN || errno == EINTR)
            {
                continue;
            }

            CHECK(len);
            break;
        }

        uint8_t* ptr = buf + (1 + HCI_EVENT_HDR_SIZE);
        len -= (1 + HCI_EVENT_HDR_SIZE);

        evt_le_meta_event* meta = reinterpret_cast<evt_le_meta_event*>(ptr);

        if (meta->subevent != 0x02)
        {
            std::cerr << "error: subevent " << static_cast<int>(meta->subevent);
            break;
        }

        le_advertising_info* info = reinterpret_cast<le_advertising_info*>(meta->data + 1);

        
        char addr[19] = {};
        ba2str(&info->bdaddr, addr);
        
        auto it = whitelist.find(std::string(addr));
        if(it == whitelist.end())
        {
            continue;
        }
        
        for(size_t i = 0; i < info->length;)
        {
            uint8_t data_len = info->data[i];
            if(data_len <= 1 || i + data_len > info->length)
            {
                break;
            }
            
           uint8_t type = info->data[i + 1];
           if(type != 0xFF || data_len != 0x10)
           {
               i += data_len + 1;
               continue;
           }

            SensorData data = {};
            memcpy(&data, info->data + i + 4, data_len - 2);

            auto t = std::time(nullptr);
            auto tm = *std::localtime(&t);

            std::ostringstream oss;
            oss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
            auto date = oss.str();
            uint32_t loss = 0;

            if(data.msg_id > it->second.last_seen_msg_id)
            {
                int32_t diff = data.msg_id - it->second.last_seen_msg_id;
                if(diff > 1 && !it->second.first_message)
                {          
                    loss = diff - 1;          
                    std::cerr << "missed " << loss << " messages for  " << it->second.name << "\n";
                }
                it->second.last_seen_msg_id = data.msg_id;
            }
            else if(data.msg_id < it->second.last_seen_msg_id)
            {
                // uint16_t overflow
                it->second.last_seen_msg_id = data.msg_id;
            }
            else if(!it->second.first_message)
            {
                break; // message already seen
            }

            it->second.first_message = false; 
            printf("[%s] %s: %.2f°C %.2f%%rH %dmV %d%%\n", date.c_str(), it->second.name.c_str(), data.temperature / 1000.f, data.humidity / 1000.f, data.battery_voltage, data.battery_percentage);
            send_to_influx(it->second, data, loss);
            break;
        }
    }

    CHECK(hci_le_set_scan_enable(device, 0, 0, 1000));
    hci_close_dev(device);
    
    curl_global_cleanup();
    return 0;

}