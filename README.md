# MotoRaiz Speedo 🏍️

GPS speedometer for the **M5Stack Cardputer-Adv** with the **Cap LoRa-1262** (LoRa + GNSS) — e.g. the Cardputer Mesh Kit.
Velocímetro GPS para o **M5Stack Cardputer-Adv** com o **Cap LoRa-1262** — por exemplo o Cardputer Mesh Kit.

> 🇵🇹 Versão em português mais abaixo.

## Features
- Big, centred speed (km/h) in portrait mode
- **5 Hz** GPS updates (CASIC commands, not saved to the module — it returns to default on power-off)
- Max speed, average speed, distance and moving time
- Local Portugal time with automatic summer/winter change
- Falls back to position-based speed if the module does not send RMC speed
- Auto-detects GPS baud rate; enables the Cap's RF/LNA switch via its I/O expander

## Keys
| Key | Action |
|-----|--------|
| `R` | Reset trip (max, average, distance, time) |
| `B` | Brightness normal / max (sunlight) |
| `O` | Rotate screen 180° |

## Install
**Easy way (M5Launcher):** download `firmware.bin` from [Releases](../../releases), copy it to the microSD card and install it from M5Launcher → SD.

**Build it yourself:** open the folder in VS Code with the PlatformIO extension and click *Build*.
On Windows, `platformio.ini` sets `core_dir = C:/pio` to avoid the 260-character path limit — change it if you prefer.

## Hardware
Tested only on **Cardputer-Adv + Cap LoRa-1262**. GNSS UART on G15 (RX) / G13 (TX).

## ⚠️ Disclaimer
Not a certified measuring device. Do not use it as evidence for speed fines. **Do not look at or operate it while riding.**

---

## 🇵🇹 Português

### Funcionalidades
- Velocidade grande e centrada (km/h), ecrã na vertical
- GPS a **5 atualizações por segundo** (comandos CASIC, não gravados no módulo)
- Velocidade máxima, média, distância e tempo em movimento
- Hora de Portugal com mudança automática verão/inverno
- Se o módulo não enviar a velocidade, é calculada pelas posições

### Teclas
`R` reset da viagem · `B` brilho · `O` rodar ecrã 180°

### Instalação
Descarrega o `firmware.bin` em [Releases](../../releases), copia para o cartão microSD e instala pelo M5Launcher → SD.

### Aviso
Não é um aparelho certificado. Não mexer nem olhar para ele em andamento.

---

## Credits
- [TinyGPSPlus](https://github.com/mikalhart/TinyGPSPlus) by Mikal Hart (LGPL-2.1)
- [M5Cardputer](https://github.com/m5stack/M5Cardputer) / M5Unified / M5GFX by M5Stack

Made in Portugal by **Moto Raiz** — parts for classic motorcycles · [motoraiz.pt](https://motoraiz.pt)

License: MIT
