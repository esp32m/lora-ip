#pragma once

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_task_wdt.h>
#include <esp_mac.h>
#include <memory>
#include <lwip/lwip_napt.h>
#include <lwip/esp_netif_net_stack.h>
#include <dhcpserver/dhcpserver.h>

#include <esp32m/logging.hpp>
#include <esp32m/net/net.hpp>
#include <esp32m/io/gpio.hpp>
#include <esp32m/bus/i2c.hpp>

#include "EspHal.h"

#include "loramesher.h"

using namespace esp32m;

class Ifbase;

struct LoraNetifDriver
{
    esp_netif_driver_base_t base;
    Ifbase *owner;
};

enum PacketDirection
{
    ServerToClient,
    ClientToServer
};

struct IfPacket
{
    PacketDirection direction;
    uint8_t data[1600];
};

static uint16_t mac2local(void *buf)
{
    auto mac = (uint8_t *)buf;
    return (mac[4] << 8) | mac[5];
}

class Ifbase : public log::Loggable
{
public:
    Ifbase(bool isServer) : _isServer(isServer)
    {
        _driver.owner = this;
        _driver.base.post_attach = [](esp_netif_t *esp_netif, void *args)
        {
            auto driver = (LoraNetifDriver *)args;
            return driver->owner->postAttach();
        };
    }
    const char *name() const override { return _isServer ? "LORA_SRV" : "LORA_CLI"; }
    esp_netif_t *handle() const { return _handle; }

    virtual esp_err_t up()
    {
        if (!_attached)
        {
            ESP_CHECK_RETURN(esp_netif_attach(_handle, &_driver));
            _attached = true;
        }
        ESP_CHECK_RETURN(esp_netif_set_mac(_handle, _mac));
        esp_netif_action_start(_handle, NULL, 0, NULL);
        return ESP_OK;
    }
    virtual esp_err_t down()
    {
        esp_netif_action_disconnected(_handle, NULL, 0, NULL);
        esp_netif_action_stop(_handle, NULL, 0, NULL);
        return ESP_OK;
    }

    void received(void *buffer, size_t len)
    {
        esp_netif_receive(_handle, buffer, len, nullptr);
    }

protected:
    bool _isServer;
    esp_netif_t *_handle;
    LoraNetifDriver _driver = {};
    bool _attached = false;
    uint8_t _mac[6];
    IfPacket _packetbuf;
    esp_err_t postAttach()
    {
        _driver.base.netif = _handle;
        esp_netif_driver_ifconfig_t ifconfig = {
            .handle = &_driver,
            .transmit = [](void *h, void *buffer, size_t len)
            {
                auto driver = (LoraNetifDriver *)h;
                return driver->owner->transmit(buffer, len); },
            .transmit_wrap = [](void *h, void *buffer, size_t len, void *netstack_buf)
            {
                auto driver = (LoraNetifDriver *)h;
                return driver->owner->transmit(buffer, len); },
            .driver_free_rx_buffer = [](void *h, void *buffer)
            { free(buffer); },
        };
        return esp_netif_set_driver_config(_handle, &ifconfig);
    }
    esp_err_t transmit(void *buffer, size_t len)
    {
        auto dest = mac2local(buffer); // first 6 bytes is dest mac
        logI("transmit %i bytes to %X", len, dest);
        if (len > 1500)
        {
            logW("packet too big: %d", len);
            return ESP_FAIL;
        }
        // this->logger().dump(log::Level::Info, buffer, len);
        size_t packetSize = 4 + len;
        // auto packet = (IfPacket*) calloc(1, packetSize);
        _packetbuf.direction = _isServer ? PacketDirection::ServerToClient : PacketDirection::ClientToServer;
        memcpy(_packetbuf.data, buffer, len);
        LoraMesher &radio = LoraMesher::getInstance();
        radio.sendReliablePacket(dest, (uint8_t *)&_packetbuf, packetSize);
        // free(packet);

        return ESP_OK;
    }
};

enum MeshMode
{
    NotInitialized = 0,
    Client = 1 << 0,
    Server = 1 << 1,
    ClientAndServer = Client | Server
};

/**
 * Internet connection not available:
 *  * LORA_SRV - absent
 *  * LORA_CLI - present
 * Non-LORA Internet connection available:
 *  * LORA_SRV - present
 *  * LORA_CLI - absent
 * LORA Internet connection available:
 *  * LORA_SRV - present
 *  * LORA_CLI - present
 */
class Ifsrv;
static Ifsrv *_sifsrv;

class Ifsrv : public Ifbase
{
public:
    Ifsrv() : Ifbase(true)
    {
        _sifsrv = this;
        esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_AP();
        base_cfg.if_key = name();
        base_cfg.if_desc = "LoRa server";
        base_cfg.ip_info = &_ipInfo;
        base_cfg.route_prio--;

        esp_netif_config_t cfg = {
            .base = &base_cfg,
            .driver = NULL,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_AP};
        _handle = esp_netif_new(&cfg);
        esp_wifi_get_mac(WIFI_IF_AP, _mac);
    }

    esp_err_t up() override
    {
        esp_netif_dns_info_t dns;
        dns.ip.u_addr.ip4.addr = 0x01010101;
        dns.ip.type = IPADDR_TYPE_V4;
        dhcps_offer_t dhcps_dns_value = OFFER_DNS;
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_stop(_handle));
        ESP_ERROR_CHECK(esp_netif_dhcps_option(_handle, ESP_NETIF_OP_SET, ESP_NETIF_DOMAIN_NAME_SERVER, &dhcps_dns_value, sizeof(dhcps_dns_value)));
        ESP_ERROR_CHECK(esp_netif_set_dns_info(_handle, ESP_NETIF_DNS_MAIN, &dns));
        ESP_ERROR_CHECK_WITHOUT_ABORT(esp_netif_dhcps_start(_handle));
        ESP_CHECK_RETURN(Ifbase::up());
        ip_napt_enable(_ipInfo.ip.addr, 1);
        return ESP_OK;
    }

private:
    esp_netif_ip_info_t _ipInfo = {
        .ip = {.addr = ESP_IP4TOADDR(10, 0, 0, 1)},
        .netmask = {.addr = ESP_IP4TOADDR(255, 255, 0, 0)},
        .gw = {.addr = ESP_IP4TOADDR(10, 0, 0, 1)},
    };
};

class Ifcli : public Ifbase
{
public:
    Ifcli() : Ifbase(false)
    {
        esp_netif_inherent_config_t base_cfg = ESP_NETIF_INHERENT_DEFAULT_WIFI_STA();
        base_cfg.if_key = name();
        base_cfg.if_desc = "LoRa client";
        base_cfg.route_prio--;

        /*auto defstack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA;
        _netstack.lwip.input_fn = defstack->lwip.input_fn;
        _netstack.lwip.init_fn = [](netif *i)
        {
            auto defstack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA;
            auto r=defstack->lwip.init_fn(i); i->mtu=100; return r; };
*/
        esp_netif_config_t cfg = {
            .base = &base_cfg,
            .driver = NULL,
            .stack = ESP_NETIF_NETSTACK_DEFAULT_WIFI_STA};
        _handle = esp_netif_new(&cfg);
        esp_wifi_get_mac(WIFI_IF_STA, _mac);
    }
    esp_err_t up() override
    {
        ESP_CHECK_RETURN(Ifbase::up());
        esp_netif_action_connected(_handle, NULL, 0, NULL);
        return ESP_OK;
    }

};

class LoraMeshNode : public AppObject
{
public:
    const char *name() const override { return "lora-mesh-node"; }
    LoraMeshNode()
    {
        xTaskCreate([](void *self)
                    { ((LoraMeshNode *)self)->run(); },
                    "loramesh", 4096, this,
                    tskIDLE_PRIORITY, &_task);
    }
    void received(AppPacket<uint8_t> *packet)
    {
        auto ifpacket = (IfPacket *)packet->payload;
        logI("Received packet from %X to %X, size %d, dir=%i", packet->src, packet->dst, packet->payloadSize - 4, ifpacket->direction);
        switch (ifpacket->direction)
        {
        case PacketDirection::ServerToClient:
            if (_ifcli)
                _ifcli->received(ifpacket->data, packet->payloadSize - 4);
            break;
        case PacketDirection::ClientToServer:
            if (_ifsrv)
                _ifsrv->received(ifpacket->data, packet->payloadSize - 4);
            break;
        default:
            logW("invalid packet direction %i", ifpacket->direction);
            break;
        }
    }

private:
    TaskHandle_t _task;
    std::unique_ptr<Ifcli> _ifcli;
    std::unique_ptr<Ifsrv> _ifsrv;
    MeshMode _mode = MeshMode::NotInitialized;
    void run()
    {
        esp_task_wdt_add(NULL);
        for (;;)
        {
            esp_task_wdt_reset();
            setMode(detectTargetMode());
            ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(1000));
        }
    }
    MeshMode detectTargetMode()
    {
        esp_netif_t *dif = esp_netif_get_default_netif();
        if (!dif)
            return MeshMode::Client;
        if (!esp_netif_is_netif_up(dif))
            return MeshMode::Client;
        auto flags = esp_netif_get_flags(dif);
        if (flags & ESP_NETIF_DHCP_SERVER)
            return MeshMode::Client;
        esp_netif_ip_info_t info;
        auto err = esp_netif_get_ip_info(dif, &info);
        if (err != ERR_OK)
            return MeshMode::Client;
        if (!info.gw.addr)
            return MeshMode::Client;

        if (_ifcli && _ifcli->handle() == dif)
            return MeshMode::ClientAndServer;
        return MeshMode::Server;
    }
    void setMode(MeshMode mode)
    {
        if (_mode == mode)
            return;
        logI("setting mode: %i -> %i", _mode, mode);
        _mode = mode;
        if (_mode & MeshMode::Client)
        {
            if (!_ifcli)
                _ifcli.reset(new Ifcli());
            _ifcli->up();
        }
        else
        {
            if (_ifcli)
                _ifcli->down();
        }
        if (_mode & MeshMode::Server)
        {
            if (!_ifsrv)
                _ifsrv.reset(new Ifsrv());
            _ifsrv->up();
        }
        else
        {
            if (_ifsrv)
                _ifsrv->down();
        }
    }
};

class LoRaMesherTest : public log::Loggable
{
public:
    LoRaMesherTest()
    {
        xTaskCreate([](void *self)
                    { ((LoRaMesherTest *)self)->run(); },
                    "lrmtest", 4096, this,
                    tskIDLE_PRIORITY + 2, &_task);
    }
    const char *name() const { return "lrmtest"; }

    const LM_LinkedList<RouteNode> *getRoutingTable()
    {
        LoraMesher &radio = LoraMesher::getInstance();
        return radio.routingTableListCopy();
    }

private:
    TaskHandle_t _task = nullptr;
    LoraMeshNode *_node = nullptr;

    /**
     * @brief This function write to the parameter routingTable the routing table of the mesh network
     */
    void printRoutingTable()
    {
        LoraMesher &radio = LoraMesher::getInstance();

        logI("Routing table");

        // Set the routing table list that is being used and cannot be accessed (Remember to release use after usage)
        auto routingTableList = radio.routingTableListCopy();

        routingTableList->setInUse();

        if (routingTableList->moveToStart())
        {
            do
            {
                auto routeNode = routingTableList->getCurrent();
                auto node = routeNode->networkNode;
                logI("Node: %X, Metric: %d, Via: %X", node.address, node.metric, routeNode->via);

            } while (routingTableList->next());
        }
        else
        {
            logI("No routes");
        }

        // Release routing table list usage.
        routingTableList->releaseInUse();
    }

    void processReceivedPackets()
    {
        LoraMesher &radio = LoraMesher::getInstance();
        for (;;)
        {
            /* Wait for the notification of processReceivedPackets and enter blocking */
            ulTaskNotifyTake(pdPASS, portMAX_DELAY);
            // led_Flash(1, 100); // one quick LED flashes to indicate a packet has arrived

            // Iterate through all the packets inside the Received User Packets FiFo
            while (radio.getReceivedQueueSize() > 0)
            {
                // logI("Fifo receiveUserData size: %d", radio.getReceivedQueueSize() > 0);

                // Get the first element inside the Received User Packets FiFo
                AppPacket<uint8_t> *packet = radio.getNextAppPacket<uint8_t>();

                // Print the data packet
                // printDataPacket(packet);

                // Print packet
                if (_node)
                    _node->received(packet);

                // printRoutingTable();

                // Delete the packet when used. It is very important to call this function to release the memory of the packet.
                radio.deletePacket(packet);
            }
        }
    }

    void run()
    {
        TaskHandle_t receiveLoRaMessage_Handle = NULL;
        LoraMesher &radio = LoraMesher::getInstance();

        LoraMesher::LoraMesherConfig config;
        config.module = LoraMesher::LoraModules::SX1262_MOD;
        radio.begin(config);
        xTaskCreate([](void *self)
                    { ((LoRaMesherTest *)self)->processReceivedPackets(); },
                    "Receive App Task", 4096, this,
                    tskIDLE_PRIORITY + 2, &receiveLoRaMessage_Handle);

        radio.setReceiveAppDataTaskHandle(receiveLoRaMessage_Handle);
        radio.start();
        logI("Lora initialized");
        _node = new LoraMeshNode();
        for (;;)
            /*{
                helloPacket->counter = dataCounter++;

                // Create packet and send it.
                radio.createPacketAndSend(BROADCAST_ADDR, helloPacket, 1);*/

            vTaskDelay(20000 / portTICK_PERIOD_MS);
        printRoutingTable();
    }
};