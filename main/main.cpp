#include <esp32m/app.hpp>
#include <esp32m/net/wifi.hpp>
#include <esp32m/net/sntp.hpp>
#include <esp32m/net/ota.hpp>
#include <esp32m/net/mqtt.hpp>
#include <esp32m/net/interfaces.hpp>

#include <esp32m/io/gpio.hpp>

#include <esp32m/dev/esp32.hpp>

#include <esp32m/ui.hpp>
#include <esp32m/ui/httpd.hpp>
#include <esp32m/ui/console.hpp>

#include <dist/ui.hpp>
#include "lrmtest.hpp"

using namespace esp32m;

extern "C" void app_main()
{
  esp_log_level_set("*", ESP_LOG_WARN);
  esp_log_level_set("LoRaMesher", ESP_LOG_VERBOSE);
  App::Init app;
  app.inferHostname();
  net::useWifi();
  net::useSntp();
  net::useMqtt();
  net::Mqtt::instance().enable(false);
  net::useInterfaces();
  net::useOta();
  dev::useEsp32();
  initUi(new Ui(new ui::Httpd()));
  new LoRaMesherTest();
}
