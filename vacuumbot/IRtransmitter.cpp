#include <pigpiod_if2.h>
#include <iostream>
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <cstdlib>
#include <unistd.h>   // per sleep()
#include <cstdio>     // per popen
#include <memory>

struct IRSignal {
    std::string type; // "pulse" o "space"
    int duration;     // microsecondi
};

const int IR_RX_PIN = 27;   // GPIO ricevitore IR
const int IR_TX_PIN = 22;   // GPIO LED IR
const int CARRIER_FREQ = 38000; // Hz (38 kHz standard)
const double DUTY_CYCLE = 0.33; // 33%
const int DELAY_BEFORE_SEND = 5; // secondi

// --- FUNZIONE PER REGISTRARE UN TASTO ---
std::vector<IRSignal> record_ir() {
    std::cout << "Premi un tasto sul telecomando..." << std::endl;
    std::vector<IRSignal> raw_signal;

    // Avvio mode2 tramite popen (lettura output)
    FILE* pipe = popen("sudo mode2 -d /dev/lirc1", "r");
    if (!pipe) {
        std::cerr << "Errore: impossibile avviare mode2" << std::endl;
        return raw_signal;
    }

    char buffer[128];
    while (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        std::string line(buffer);
        if (line.find("pulse") == 0 || line.find("space") == 0) {
            std::istringstream iss(line);
            std::string type;
            int length;
            iss >> type >> length;
            raw_signal.push_back({type, length});
        } else if (line.find("timeout") == 0 && !raw_signal.empty()) {
            if (!raw_signal.empty()) {
                std::cout << "\nRaw array catturato:" << std::endl;
                for (auto &s : raw_signal) {
                    std::cout << "(" << s.type << ", " << s.duration << ") ";
                }
		        std::cout << std::endl << std::flush;

		        system("sudo pkill mode2"); 
		        pclose(pipe);

                std::ofstream outfile("raw_ir.txt");
                if (outfile.is_open()) {
                    for (const auto& s : raw_signal) {
                        outfile << s.type << " " << s.duration << "\n";
                    }
                    outfile.close();
                    std::cout << "Segnale IR salvato su raw_ir.txt" << std::endl;
                } else {
                    std::cerr << "Errore: impossibile aprire il file per scrittura" << std::endl;
                }

                break;
            }
        }
    }
    return raw_signal;
}

// --- FUNZIONE PER TRASMETTERE ---
void send_raw_wave(int pi, const std::vector<IRSignal>& signal,
                   int gpio_pin, int carrier_freq, double duty_cycle) {
    int micros_per_cycle = static_cast<int>(1'000'000 / carrier_freq);
    int on_micros = static_cast<int>(micros_per_cycle * duty_cycle);
    int off_micros = micros_per_cycle - on_micros;

    std::vector<gpioPulse_t> wf;

    for (const auto& s : signal) {
        if (s.type == "pulse") {
            int cycles = s.duration / micros_per_cycle;
            for (int i = 0; i < cycles; i++) {
                gpioPulse_t p1 = {1u << gpio_pin, 0, static_cast<uint32_t>(on_micros)};
                gpioPulse_t p2 = {0, 1u << gpio_pin, static_cast<uint32_t>(off_micros)};
                wf.push_back(p1);
                wf.push_back(p2);
            }
        } else if (s.type == "space") {
            gpioPulse_t space = {0, 0, static_cast<uint32_t>(s.duration)};
            wf.push_back(space);
        }
    }

    wave_clear(pi);
    wave_add_generic(pi, wf.size(), wf.data());
    int wave_id = wave_create(pi);

    if (wave_id >= 0) {
        wave_send_once(pi, wave_id);
        while (wave_tx_busy(pi)) {
            time_sleep(0.001); // attesa fine trasmissione
        }
        wave_delete(pi, wave_id);
    }
}

int main() {
    // Connessione al demone pigpiod
    int pi = pigpio_start(NULL, NULL); // NULL,NULL = locale
    if (pi < 0) {
        std::cerr << "Errore: impossibile connettersi al demone pigpiod" << std::endl;
        return 1;
    }

    // --- Registrazione segnale IR ---
    std::vector<IRSignal> raw_array = record_ir();

    if (raw_array.empty()) {
        std::cerr << "Nessun segnale catturato." << std::endl;
        pigpio_stop(pi);
        return 1;
    }

    std::cout << "\nAspetto " << DELAY_BEFORE_SEND << " secondi prima di trasmettere..." << std::endl;
    sleep(DELAY_BEFORE_SEND);

    std::cout << "Trasmissione del segnale catturato..." << std::endl;
    send_raw_wave(pi, raw_array, IR_TX_PIN, CARRIER_FREQ, DUTY_CYCLE);
    std::cout << "Trasmissione completata!" << std::endl;

    pigpio_stop(pi); // disconnessione
    return 0;
}
