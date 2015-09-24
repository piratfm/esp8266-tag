/*
 *  Simple mac tracking tag
 */


#include "user_config.h"
#include "osapi.h"
#include "user_interface.h"
#include "espconn.h"
#include "driver/uart.h"
#include "ping.h"
#include "mem.h"

#include "lwip/def.h"
#include "lwip/mem.h"
#include "etharp.h"

#define user_procTaskPrio        0
#define user_procTaskQueueLen    1
os_event_t    user_procTaskQueue[user_procTaskQueueLen];
static void loop(os_event_t *events);
void ICACHE_FLASH_ATTR network_send_stat();

struct ping_option pingopts;


static const uint8_t mac_to_check[6] = MAC_TO_FIND;

uint8_t network_retry_count=0;
uint8_t nslookup_retry_count=0;
uint8_t stats_sent=0;


uint8_t ping_is_running=1;
uint32_t ip_addr_local=0;
uint32_t ip_addr_start=0;
uint32_t ip_addr_stop=0;

uint8_t is_alive=0;
uint8_t is_sleep=0;


uint32_t target_seen_times = 0;
uint32_t target_not_found_times = 0;


typedef struct stats_buff_s {
    uint8_t flag;
    uint32_t ip;
    uint8_t mac[6];
} stats_buff_t;


stats_buff_t *alive_ips = NULL;
uint16_t alive_ips_num = 0;

static void ICACHE_FLASH_ATTR my_dev_not_found(void)
{
    target_not_found_times++;
    int ret = system_rtc_mem_write (66, &target_not_found_times, 4);

}

static int dev_is_found = 0;
static void ICACHE_FLASH_ATTR save_my_dev(uint16_t ip_id)
{
    uint8_t mac_octet, not_found=0;
    for(mac_octet=0;mac_octet<6;mac_octet++)
        if(alive_ips[ip_id].mac[mac_octet] != mac_to_check[mac_octet])
            not_found=1;

    if(not_found) return;

    os_printf(", IS FOUND!");
    dev_is_found = 1;
    int ret = system_rtc_mem_write (65, &alive_ips[ip_id].ip, 4);
    target_seen_times++;
    ret = system_rtc_mem_write (64, &target_seen_times, 4);
    target_not_found_times = 0;
    ret = system_rtc_mem_write (66, &target_not_found_times, 4);
    //finish searching by setting last ip...
    ip_addr_stop = ip_addr_start+1;
}




//SAMPLE: https://github.com/willemwouters/ESP8266/blob/master/sdk/esp_iot_sdk_v0.9.3/IoT_Demo/lwip/app/ping.c
//source: https://github.com/CHERTS/esp8266-liblwip
//sample2: https://github.com/nekromant/esp8266-frankenstein/blob/master/src/cmd_ping.c
//sample3: http://bbs.espressif.com/viewtopic.php?t=583
void ICACHE_FLASH_ATTR
user_ping_sent(void *arg, void *pdata)
{
    ip_addr_t ipaddr;
    struct eth_addr *ret_eth_addr = NULL;
    struct ip_addr *ret_ip_addr = NULL;

    ipaddr.addr = htonl(ip_addr_start);
    int arp_find = etharp_find_addr(netif_default, &ipaddr, &ret_eth_addr, &ret_ip_addr);

    if(arp_find != -1 && ret_eth_addr != NULL) {
      os_printf("MAC:%02x:%02x:%02x:%02x:%02x:%02x",
      ret_eth_addr->addr[0], ret_eth_addr->addr[1], ret_eth_addr->addr[2], ret_eth_addr->addr[3], ret_eth_addr->addr[4], ret_eth_addr->addr[5]);
    }

    if(is_alive || (arp_find != -1 && ret_eth_addr!=NULL)){
      os_printf(" ALIVE: a:%d p:%d", ret_eth_addr ? 1 : 0, is_alive ? 1 : 0);

      alive_ips = (stats_buff_t *) os_realloc(alive_ips, sizeof(struct stats_buff_s) * (alive_ips_num+1));
      alive_ips[alive_ips_num].ip = ip_addr_start;
      alive_ips[alive_ips_num].flag = (ret_eth_addr ? 1 : 0) | (is_alive ? 2 : 0);

      int i;
      for (i=0; i<6;i++) {
          alive_ips[alive_ips_num].mac[i] = (arp_find != -1 && ret_eth_addr!=NULL) ? ret_eth_addr->addr[i] : 0;
      }

      save_my_dev(alive_ips_num);

      alive_ips_num++;
      is_alive=0;
    }
    os_printf("\r\n");
    ip_addr_start++;
    ping_is_running=0;
}

static void ICACHE_FLASH_ATTR ping_recv_callback(void* arg, void *pdata)
{
  struct ping_resp *pingresp = (struct ping_resp*) pdata;

 if(pingresp->ping_err == 0 && pingresp->bytes > 0)
    is_alive=1;
}

static int ICACHE_FLASH_ATTR do_ping_new(const uint32_t ip_uint)
{
  ping_is_running=1;
  os_printf("ping: %u.%d.%d.%d...", 0x00ff &(ip_uint >> 24), 0x00ff &(ip_uint >> 16), 0x00ff &(ip_uint >> 8), 0x00ff & ip_uint);

  is_alive=0;
  memset(&pingopts, 0x00, sizeof(struct ping_option));
  pingopts.ip = htonl(ip_uint);
  pingopts.count = 2;
  if(target_not_found_times==0) {
    pingopts.coarse_time = 10;  // 1000 ms
  } else {
    pingopts.coarse_time = 1;  // 100 ms
  }

  pingopts.recv_function=ping_recv_callback;
  pingopts.sent_function=user_ping_sent;

  ping_start(&pingopts);
  return 0;
}







//Main code function
static void ICACHE_FLASH_ATTR
loop(os_event_t *events)
{

    os_delay_us(10000);
    system_os_post(user_procTaskPrio, 0, 0 );
    if(is_sleep) { os_printf("x"); return; }


    if(network_retry_count > 10 || nslookup_retry_count > 5 || stats_sent > 0) {
        os_printf("Sleeping deeply...\r\n");
        if(( target_not_found_times > 0 && target_seen_times == 0 && (target_not_found_times % FULL_SCAN_PERIOD == 0)) ||
           (target_seen_times > 0 && (target_seen_times % FULL_SCAN_PERIOD == 0))) {
            os_printf("Next start will be with RF calibration\r\n");
            system_deep_sleep_set_option(0);
        } else {
            system_deep_sleep_set_option(3);
        }

        if(target_not_found_times > 10) {
            system_deep_sleep(DEEP_SLEEP_TIME_HIGH);
        } else {
            system_deep_sleep(DEEP_SLEEP_TIME);
        }
        is_sleep=1;
        return;
    }

    if(ping_is_running) { return; }

    if(ip_addr_start < ip_addr_stop) {
        if(ip_addr_start == ip_addr_local) ip_addr_start++;
        do_ping_new(ip_addr_start);
    } else if(ip_addr_start == ip_addr_stop) {
        if(!dev_is_found)
            my_dev_not_found();
        os_delay_us(50000);
        network_send_stat();
        ip_addr_start++;
    } else {
        os_printf("X");
    }
}

static void ICACHE_FLASH_ATTR networkConnectedCb(void *arg);
static void ICACHE_FLASH_ATTR networkDisconCb(void *arg);
static void ICACHE_FLASH_ATTR networkReconCb(void *arg, sint8 err);
static void ICACHE_FLASH_ATTR networkRecvCb(void *arg, char *data, unsigned short len);
static void ICACHE_FLASH_ATTR networkSentCb(void *arg);
void ICACHE_FLASH_ATTR network_init();
 
LOCAL os_timer_t network_timer;
 
static void ICACHE_FLASH_ATTR networkServerFoundCb(const char *name, ip_addr_t *ip, void *arg) {
  static esp_tcp tcp;
  struct espconn *conn=(struct espconn *)arg;
  if (ip==NULL) {
    os_printf("Nslookup failed :/ Trying again...\r\n");
    nslookup_retry_count++;
    network_init();
  }

  os_printf("lokk\r\n",4);
  char page_buffer[20];
  os_sprintf(page_buffer,"DST: %d.%d.%d.%d\r\n",
  *((uint8 *)&ip->addr), *((uint8 *)&ip->addr + 1),
  *((uint8 *)&ip->addr + 2), *((uint8 *)&ip->addr + 3));
  os_printf(page_buffer,strlen(page_buffer));

  conn->type=ESPCONN_TCP;
  conn->state=ESPCONN_NONE;
  conn->proto.tcp=&tcp;
  conn->proto.tcp->local_port=espconn_port();
  conn->proto.tcp->remote_port=80;
  os_memcpy(conn->proto.tcp->remote_ip, &ip->addr, 4);
  espconn_regist_connectcb(conn, networkConnectedCb);
  espconn_regist_disconcb(conn, networkDisconCb);
  espconn_regist_reconcb(conn, networkReconCb);
  espconn_regist_recvcb(conn, networkRecvCb);
  espconn_regist_sentcb(conn, networkSentCb);
  espconn_connect(conn);
}

static void ICACHE_FLASH_ATTR networkSentCb(void *arg) {
  os_printf("sent\r\n");
}

static void ICACHE_FLASH_ATTR networkRecvCb(void *arg, char *data, unsigned short len) {

  os_printf("recv\r\n");

  struct espconn *conn=(struct espconn *)arg;
  int x;
  os_printf(data);
  stats_sent++;
  espconn_disconnect(conn);
}

static void ICACHE_FLASH_ATTR networkConnectedCb(void *arg) {

  os_printf("conn\r\n");
  struct espconn *conn=(struct espconn *)arg;

  char *data = (char *)os_malloc(alive_ips_num * (4*3 + 3 + 1 + 7*3) + 256);

  os_sprintf(data,"GET /input/" TRACKING_PUBKEY "?private_key=" TRACKING_PRIVKEY "&presence=%d,%d,%d&existance=", target_seen_times, target_not_found_times, readvdd33());

  uint16_t ip_id;
  for(ip_id=0; ip_id < alive_ips_num; ip_id++) {
    os_sprintf(data+strlen(data),"%02x,%02x:%02x:%02x:%02x:%02x:%02x,%u.%u.%u.%u%s",
    alive_ips[ip_id].flag,
    alive_ips[ip_id].mac[0], alive_ips[ip_id].mac[1], alive_ips[ip_id].mac[2], alive_ips[ip_id].mac[3], alive_ips[ip_id].mac[4], alive_ips[ip_id].mac[5],
     0x00ff &(alive_ips[ip_id].ip >> 24), 0x00ff &(alive_ips[ip_id].ip >> 16), 0x00ff &(alive_ips[ip_id].ip >> 8), 0x00ff & alive_ips[ip_id].ip,
    ip_id == alive_ips_num-1 ? "" : "|");

  }
  os_sprintf(data+strlen(data)," HTTP/1.0\r\n"
               "Host: " TRACKING_PHANT_HOST "\r\n"
               "Connection: close\r\n"
               "\r\n\r\n");

  os_printf(data,strlen(data));
  sint8 d = espconn_sent(conn,data,strlen(data));
 
//  os_free(tmp);
  espconn_regist_recvcb(conn, networkRecvCb);
  os_printf("send\r\n");
}
 
static void ICACHE_FLASH_ATTR networkReconCb(void *arg, sint8 err) {
  os_printf("Reconnect\n\r");
  stats_sent++;
//  network_init();
}
 
static void ICACHE_FLASH_ATTR networkDisconCb(void *arg) {
  os_printf("Disconnect\n\r");
  stats_sent++;
//  network_init();
}
 
 
void ICACHE_FLASH_ATTR network_send_stat() {
  static struct espconn conn;
  static ip_addr_t ip;
  os_printf("Looking up server...\r\n");
    os_printf("look\r\n");
  espconn_gethostbyname(&conn, TRACKING_PHANT_HOST, &ip, networkServerFoundCb);
}

void ICACHE_FLASH_ATTR network_check_ip(void) {
  struct ip_info ipconfig;
  os_timer_disarm(&network_timer);
  wifi_get_ip_info(STATION_IF, &ipconfig);
  if (wifi_station_get_connect_status() == STATION_GOT_IP && ipconfig.ip.addr != 0) {
    char page_buffer[20];
    os_sprintf(page_buffer,"IP: %d.%d.%d.%d\r\n",IP2STR(&ipconfig.ip));
    os_printf(page_buffer,strlen(page_buffer));
    ip_addr_local = ntohl(ipconfig.ip.addr);

    if(target_seen_times == 0 || (target_not_found_times != 0 && (target_not_found_times % FULL_SCAN_PERIOD == 0))) {
        os_printf("Full net scan\n\r", network_retry_count);
        ip_addr_start = ntohl(ipconfig.ip.addr & ipconfig.netmask.addr);
        ip_addr_stop = ip_addr_start | ~ntohl(ipconfig.netmask.addr);
        if (ip_addr_stop > ip_addr_start) ip_addr_start++;
        //DEBUG:
        //ip_addr_stop -= 200;
    } else {
        int ret = system_rtc_mem_read (65, &ip_addr_start, 4);
        ip_addr_stop = ip_addr_start+1;
    }

    ping_is_running=0;
  } else {
    os_printf("No ip found %d\n\r", network_retry_count);
    network_retry_count++;
    os_timer_setfn(&network_timer, (os_timer_func_t *)network_check_ip, NULL);
    os_timer_arm(&network_timer, 1000, 0);
  }
}

void ICACHE_FLASH_ATTR network_init() {
  os_timer_disarm(&network_timer);
  os_timer_setfn(&network_timer, (os_timer_func_t *)network_check_ip, NULL);
  os_timer_arm(&network_timer, 1000, 0);
}

//Init function
void ICACHE_FLASH_ATTR user_init() {

    uart_div_modify(0, UART_CLK_FREQ / BIT_RATE_115200);
    char ssid[32] = SSID;
    char password[64] = SSID_PASSWORD;

    uint32_t is_initialized=0;
    int ret = system_rtc_mem_read (67, &is_initialized, 4);
    if(is_initialized!=0xb00bface) {
        os_printf("Init RTC ram...");
        target_seen_times = 0;
        target_not_found_times = 1;
        is_initialized=0xb00bface;
        ret = system_rtc_mem_write (64, &target_seen_times, 4);
        ret = system_rtc_mem_write (66, &target_not_found_times, 4);
        ret = system_rtc_mem_write (67, &is_initialized, 4);
    }
    ret = system_rtc_mem_read (64, &target_seen_times, 4);
    ret = system_rtc_mem_read (66, &target_not_found_times, 4);
    os_printf("target_seen_times = %d | target_not_found_times = %d\r\n", target_seen_times, target_not_found_times);


    struct station_config stationConf;

    //Set station mode
    wifi_set_opmode( 0x1 );

    //Set ap settings
    os_memcpy(&stationConf.ssid, ssid, 32);
    os_memcpy(&stationConf.password, password, 64);
    wifi_station_set_config(&stationConf);

    os_printf("init\r\n");

    //Start os task
    system_os_task(loop, user_procTaskPrio,user_procTaskQueue, user_procTaskQueueLen);

    system_os_post(user_procTaskPrio, 0, 0 );

    network_init();
}
