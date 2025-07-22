#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include "aht20.h"
#include "bmp280.h"
#include "ssd1306.h"
#include "font.h"
#include <math.h>
#include "lwip/tcp.h"
#include "pico/cyw43_arch.h"
#include "lib/buzzer.h"

#define WIFI_SSID "Wifi Lipe"
#define WIFI_PASSWORD "loukikolipe"
#define BUZZER_PIN 21
#define I2C_PORT i2c0               // i2c0 pinos 0 e 1, i2c1 pinos 2 e 3
#define I2C_SDA 0                   // 0 ou 2
#define I2C_SCL 1                   // 1 ou 3
#define SEA_LEVEL_PRESSURE 101325.0 // Pressão ao nível do mar em Pa
// Display na I2C
#define I2C_PORT_DISP i2c1
#define I2C_SDA_DISP 14
#define I2C_SCL_DISP 15
#define endereco 0x3C
#define LED_ALARME_PIN 13
volatile float g_limite_temp = 30.0;
volatile float g_temp_bmp = -1.0;
volatile float g_temp_aht = -1.0;
volatile float g_umid = -1.0;
volatile float g_alt = -1.0;
const char HTML_BODY[] =
"<!DOCTYPE html><html><head><meta charset='UTF-8'><title>Monitoramento</title>"
"<style>"
"body { font-family: sans-serif; text-align: center; background: #f0f0f0; padding: 20px; }"
".container { display: flex; justify-content: center; gap: 50px; margin-top: 20px; }"
".sensor-column { background: white; padding: 20px; border-radius: 10px; box-shadow: 0 2px 5px rgba(0,0,0,0.1); min-width: 200px; }"
".dados { font-size: 24px; margin: 15px; text-align: left; }"
".sensor-title { font-weight: bold; margin-bottom: 15px; font-size: 20px; }"
".grafico-titulo { font-size: 14px; font-weight: bold; margin-bottom: 5px; color: #333; text-align: center; }"
".config-section { margin-top: 20px; padding: 10px; background: white; border-radius: 10px; display: inline-block; }"
"canvas { margin-top: 5px; border: 1px solid #ccc; background: white; }"
"input, button { padding: 5px; margin: 5px; }"
"</style>"
"<script>"
"let dados = []; let ctx = null;"
"function atualizar() {"
" fetch('/dados').then(res => res.json()).then(data => {"
"  let temp = (data.temp_bmp >= 0) ? data.temp_bmp : (data.temp_aht >= 0 ? data.temp_aht : null);"
"  document.getElementById('temp_bmp').innerText = data.temp_bmp >= 0 ? data.temp_bmp.toFixed(1) + ' °C' : '--';"
"  document.getElementById('alt').innerText = data.alt >= 0 ? data.alt.toFixed(1) + ' m' : '--';"
"  document.getElementById('temp_aht').innerText = data.temp_aht >= 0 ? data.temp_aht.toFixed(1) + ' °C' : '--';"
"  document.getElementById('umid').innerText = data.umid >= 0 ? data.umid.toFixed(1) + ' %' : '--';"
"  if (temp !== null) atualizarGrafico(temp);"
"}).catch(err => {"
"  document.getElementById('temp_bmp').innerText = '--';"
"  document.getElementById('alt').innerText = '--';"
"  document.getElementById('temp_aht').innerText = '--';"
"  document.getElementById('umid').innerText = '--';"
"});"
"}"
"function atualizarGrafico(valor) {"
"  const minTemp = 15;"
"  const maxTemp = 40;"
"  if (dados.length >= 30) dados.shift();"
"  dados.push(valor);"
"  if (ctx) {"
"    ctx.clearRect(0, 0, 300, 100);"
"    ctx.beginPath();"
"    function mapTempY(temp) {"
"      if (temp < minTemp) temp = minTemp;"
"      if (temp > maxTemp) temp = maxTemp;"
"      return 100 - ((temp - minTemp) / (maxTemp - minTemp)) * 100;"
"    }"
"    ctx.moveTo(0, mapTempY(dados[0]));"
"    for (let i = 1; i < dados.length; i++) {"
"      ctx.lineTo(i * 10, mapTempY(dados[i]));"
"    }"
"    ctx.stroke();"
"  }"
"}"
"function definirLimite() {"
"  const limite = document.getElementById('limite').value;"
"  fetch('/set_limite?valor=' + limite);"
"}"
"window.onload = function() {"
"  ctx = document.getElementById('grafico').getContext('2d');"
"  setInterval(atualizar, 2000);"
"  atualizar();"
"}"
"</script></head><body>"
"<h1>Monitoramento de Ambiente</h1>"
"<div class='container'>"
"<div class='sensor-column'>"
"<div class='sensor-title'>BMP280</div>"
"<div class='dados'>Temperatura: <span id='temp_bmp'>--</span></div>"
"<div class='dados'>Altitude: <span id='alt'>--</span></div>"
"</div>"
"<div class='sensor-column'>"
"<div class='sensor-title'>AHT20</div>"
"<div class='dados'>Temperatura: <span id='temp_aht'>--</span></div>"
"<div class='dados'>Umidade: <span id='umid'>--</span></div>"
"</div>"
"</div>"
"<div class='grafico-titulo'>Gráfico da Temperatura (°C)</div>"
"<canvas id='grafico' width='300' height='100'></canvas>"
"<div class='config-section'>"
"<h3>Limite de Temperatura</h3>"
"<input type='number' id='limite' min='0' max='50' step='0.1' value='30.0' placeholder='Limite de temperatura'>"
"<button onclick='definirLimite()'>Definir Limite</button>"
"</div>"
"</body></html>";

// ======== HTTP SERVER ==========
struct http_state {
    char response[4096];
    size_t len;
    size_t sent;
};

static err_t http_sent(void *arg, struct tcp_pcb *tpcb, u16_t len)
{
    struct http_state *hs = (struct http_state *)arg;
    hs->sent += len;
    if (hs->sent >= hs->len) {
        tcp_close(tpcb);
        free(hs);
    }
    return ERR_OK;
}

static err_t http_recv(void *arg, struct tcp_pcb *tpcb, struct pbuf *p, err_t err)
{
    if (!p) {
        tcp_close(tpcb);
        return ERR_OK;
    }

    char *req = (char *)p->payload;
    struct http_state *hs = malloc(sizeof(struct http_state));
    if (!hs) {
        pbuf_free(p);
        tcp_close(tpcb);
        return ERR_MEM;
    }
    hs->sent = 0;

        if (strncmp(req, "GET /dados", strlen("GET /dados")) == 0) {
        char json[128];
        int json_len = snprintf(json, sizeof(json),
            "{\"temp_bmp\":%.2f,\"temp_aht\":%.2f,\"umid\":%.2f,\"alt\":%.2f}", 
            g_temp_bmp, g_temp_aht, g_umid, g_alt);

        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: application/json\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            json_len, json);
    }
    else if (strncmp(req, "GET /set_limite", strlen("GET /set_limite")) == 0) {
        char *valor_str = strstr(req, "valor=");
        if (valor_str) {
            valor_str += 6; // Pula "valor="
            float novo_limite = atof(valor_str);
            if (novo_limite >= 0 && novo_limite <= 50) {
                g_limite_temp = novo_limite;
                hs->len = snprintf(hs->response, sizeof(hs->response),
                    "HTTP/1.1 200 OK\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
            }
        }
    }
    else if (strncmp(req, "GET / ", 6) == 0 || strncmp(req, "GET /HTTP", 9) == 0 || strncmp(req, "GET / HTTP", 10) == 0) {
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 200 OK\r\nContent-Type: text/html\r\nContent-Length: %d\r\nConnection: close\r\n\r\n%s",
            (int)strlen(HTML_BODY), HTML_BODY);
    } else {
        hs->len = snprintf(hs->response, sizeof(hs->response),
            "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n");
    }

    pbuf_free(p);
    tcp_arg(tpcb, hs);
    tcp_sent(tpcb, http_sent);
    tcp_write(tpcb, hs->response, hs->len, TCP_WRITE_FLAG_COPY);
    tcp_output(tpcb);
    return ERR_OK;
}

static err_t connection_callback(void *arg, struct tcp_pcb *newpcb, err_t err)
{
    tcp_recv(newpcb, http_recv);
    return ERR_OK;
}

static void start_http_server(void)
{
    struct tcp_pcb *pcb = tcp_new();
    if (!pcb) return;
    if (tcp_bind(pcb, IP_ADDR_ANY, 80) != ERR_OK) {
        tcp_close(pcb);
        return;
    }
    pcb = tcp_listen(pcb);
    if (!pcb) return;
    tcp_accept(pcb, connection_callback);
}

// Função para calcular a altitude a partir da pressão atmosférica
double calculate_altitude(double pressure)
{
    return 44330.0 * (1.0 - pow(pressure / SEA_LEVEL_PRESSURE, 0.1903));
}

// Trecho para modo BOOTSEL com botão B
#include "pico/bootrom.h"
#define botaoB 6
void gpio_irq_handler(uint gpio, uint32_t events)
{
    reset_usb_boot(0, 0);
}

bool bmp280_presente() {
    uint8_t id;
    // Tenta ler o registrador de ID (0xD0)
    int result = i2c_write_blocking(I2C_PORT, 0x76, (uint8_t[]){0xD0}, 1, true);
    result += i2c_read_blocking(I2C_PORT, 0x76, &id, 1, false);
    return (result == 2 && id == 0x58);  // ID esperado do BMP280
}
void verificar_alarme(float temperatura) {
    static bool alarme_ativo = false;
    static absolute_time_t last_beep_time = 0;
    
    if (temperatura > g_limite_temp) {
        gpio_put(LED_ALARME_PIN, 1); // Acende o LED
        
        if (!alarme_ativo) {
            alarme_ativo = true;
            last_beep_time = get_absolute_time(); // Começa o primeiro beep imediatamente
        }
        
        // Toca um beep a cada 1 segundo (500ms ligado, 500ms desligado)
        if (absolute_time_diff_us(last_beep_time, get_absolute_time()) > 1000000) {
            buzzer_beep(BUZZER_PIN, 1000, 500); // Beep de 1kHz por 500ms
            last_beep_time = get_absolute_time();
        }
    } else {
        gpio_put(LED_ALARME_PIN, 0); // Apaga o LED
        if (alarme_ativo) {
            buzzer_stop(pwm_gpio_to_slice_num(BUZZER_PIN));
            alarme_ativo = false;
        }
    }
}
int main()
{
    // Para ser utilizado o modo BOOTSEL com botão B
    gpio_init(botaoB);
    gpio_set_dir(botaoB, GPIO_IN);
    gpio_pull_up(botaoB);
    gpio_set_irq_enabled_with_callback(botaoB, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler);

    // Fim do trecho para modo BOOTSEL com botão B
    gpio_init(LED_ALARME_PIN);
    gpio_set_dir(LED_ALARME_PIN, GPIO_OUT);
    gpio_put(LED_ALARME_PIN, 0);
    stdio_init_all();
    buzzer_init(BUZZER_PIN);
     uint slice_num_buzzer = pwm_gpio_to_slice_num(BUZZER_PIN);     
    // I2C do Display funcionando em 400Khz.
    i2c_init(I2C_PORT_DISP, 400 * 1000);

    gpio_set_function(I2C_SDA_DISP, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_DISP, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_DISP);
    gpio_pull_up(I2C_SCL_DISP);
    ssd1306_t ssd;
    ssd1306_init(&ssd, WIDTH, HEIGHT, false, endereco, I2C_PORT_DISP);
    ssd1306_config(&ssd);
    ssd1306_send_data(&ssd);

    // Limpa o display
    ssd1306_fill(&ssd, false);
    ssd1306_send_data(&ssd);

    // Inicializa o I2C
    i2c_init(I2C_PORT, 400 * 1000);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA);
    gpio_pull_up(I2C_SCL);

    // Inicializa o BMP280
    bool bmp280_ok = bmp280_presente();
    struct bmp280_calib_param params;

    if (bmp280_ok) {
        bmp280_init(I2C_PORT);
        bmp280_get_calib_params(I2C_PORT, &params);
        printf("BMP280 detectado.\n");
    } else {
        printf("BMP280 não detectado.\n");
    }

    // Inicializa o AHT20
    aht20_reset(I2C_PORT);
    aht20_init(I2C_PORT);

    if (cyw43_arch_init()) {
        printf("Erro ao iniciar CYW43\n");
        return 1;
    }

    cyw43_arch_enable_sta_mode();
    printf("Conectando ao Wi-Fi...\n");
    while (cyw43_arch_wifi_connect_timeout_ms(WIFI_SSID, WIFI_PASSWORD, CYW43_AUTH_WPA2_AES_PSK, 20000)) {
        printf("Falha ao conectar ao Wi-Fi\n");
        sleep_ms(100);
        return -1;
    }
    printf("Conectado ao Wi-Fi\n");
    if (netif_default) {
        printf("IP do dispositivo: %s\n", ipaddr_ntoa(&netif_default->ip_addr));
    }

    start_http_server();
    printf("Servidor HTTP iniciado\n");

    // Estrutura para armazenar os dados do sensor
    AHT20_Data data;
    int32_t raw_temp_bmp;
    int32_t raw_pressure;

    char str_tmp1[5];  // Buffer para armazenar a string
    char str_alt[5];   // Buffer para armazenar a string  
    char str_tmp2[5];  // Buffer para armazenar a string
    char str_umi[5];   // Buffer para armazenar a string      

    bool cor = true;
while (1)
{
    cyw43_arch_poll();
    int32_t temperature = 0;
    double altitude = 0.0;
    
    // Leitura do BMP280
    if (bmp280_ok) {
        bmp280_read_raw(I2C_PORT, &raw_temp_bmp, &raw_pressure);
        temperature = bmp280_convert_temp(raw_temp_bmp, &params);
        int32_t pressure = bmp280_convert_pressure(raw_pressure, raw_temp_bmp, &params);
        altitude = calculate_altitude(pressure);

        printf("Pressao = %.3f kPa\n", pressure / 1000.0);
        printf("Temperatura BMP: = %.2f C\n", temperature / 100.0);
        printf("Altitude estimada: %.2f m\n", altitude);
        
        g_temp_bmp = temperature / 100.0;
        g_alt = altitude;
        
        sprintf(str_tmp1, "%.1fC", temperature / 100.0);
        sprintf(str_alt, "%.0fm", altitude);
        
        // Verifica o alarme com a temperatura do BMP280
        verificar_alarme(g_temp_bmp);
    } else {
        strcpy(str_tmp1, "--");
        strcpy(str_alt, "--");
        g_temp_bmp = -1.0;
        g_alt = -1.0;
    }

    // Leitura do AHT20
    if (aht20_read(I2C_PORT, &data)) {
        printf("Temperatura AHT: %.2f C\n", data.temperature);
        printf("Umidade: %.2f %%\n\n\n", data.humidity);
        
        g_temp_aht = data.temperature;
        g_umid = data.humidity;
        
        sprintf(str_tmp2, "%.1fC", data.temperature);
        sprintf(str_umi, "%.1f%%", data.humidity);
        
        // Se o BMP280 não estiver disponível, verifica o alarme com a temperatura do AHT20
        if (!bmp280_ok) {
            verificar_alarme(g_temp_aht);
        }
    } else {
        printf("Erro na leitura do AHT10!\n\n\n");
        strcpy(str_tmp2, "--");
        strcpy(str_umi, "--");
        g_temp_aht = -1.0;
        g_umid = -1.0;
    }

        // Atualiza o conteúdo do display com animações
        ssd1306_fill(&ssd, !cor);
        ssd1306_rect(&ssd, 3, 3, 122, 60, cor, !cor);
        ssd1306_line(&ssd, 3, 25, 123, 25, cor);
        ssd1306_line(&ssd, 3, 37, 123, 37, cor);
        ssd1306_draw_string(&ssd, "CEPEDI   TIC37", 8, 6);
        ssd1306_draw_string(&ssd, "EMBARCATECH", 20, 16);
        ssd1306_draw_string(&ssd, "BMP280  AHT10", 10, 28);
        ssd1306_line(&ssd, 63, 25, 63, 60, cor);
        ssd1306_draw_string(&ssd, str_tmp1, 14, 41);
        ssd1306_draw_string(&ssd, str_alt, 14, 52);
        ssd1306_draw_string(&ssd, str_tmp2, 73, 41);
        ssd1306_draw_string(&ssd, str_umi, 73, 52);
        ssd1306_send_data(&ssd);

        sleep_ms(500);
    }
    cyw43_arch_deinit();
    return 0;
}