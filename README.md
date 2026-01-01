# SteRoPiano – STM32 + VMPK + Qsynth


Celem projektu jest stworzenie urządzenia typu MIDI umożliwiającego pracę w dwóch trybach.
1. Użytkownik naciska przyciski na klawiaturze, a przypisane do nich dźwięki są słyszalne przez urządzenie audio podłączone do płytki.
2. Zamiast wykorzystywania urządzenia audio, muzykę słychać z komputera. W tej wersji uruchomiony
zostaje także graficzny interfejs modyfikacji dzwięków w aplikacji Qsynth oraz wizualizacja pianina w VMPK 

---

## Wymagania

### Sprzęt
- STM32L476G-Discovery
- klawiatura PS/2
- głośnik
- PC z Linuxem (GNOME)

### IDE
- **STM32CubeIDE 1.19.0**

---

## Instalacja (Ubuntu / Debian GNOME)

1. Nadaj prawa wykonywania skryptowi:
   ```bash
   chmod u+x setup.sh
   ```

2. Uruchom instalację:
   ```bash
   ./setup.sh
   ```

   Skrypt:
   - instaluje `qsynth`, `vmpk`, `fluid-soundfont-gm` (APT),
   - ustawia skrót klawiaturowy **F12**,
   - przygotowuje skrypt startowy `boot_api.sh`.

3. uruchom Qsynth i zmień backend audio:

   ```text
   Qsynth → Setup → Audio → Audio Driver → alsa
   ```

   Zapisz ustawienia i zamknij Qsynth.

4. Wtedy:
   - naciśnięcie **F12** uruchamia Qsynth + VMPK

---

## Tryby pracy urządzenia

### MODE_SYNTH (domyślny)
- lokalna synteza dźwięku na STM32
- wyjście audio: DAC (CS43L22)
- zielona dioda LED

### MODE_HID
- STM32 działa jako **klawiatura USB HID**
- wysyła nuty do VMPK
- czerwona dioda LED
- automatycznie wysyłany skrót **F12** przy przełączeniu

Przełączanie trybu:
- przycisk **JOY_CENTER**

---

## Mapowanie klawiszy (PS/2 → nuty)

### Oktawa 2
- Q W E R T Y U

### Oktawa 3
- A S D F G H J

### Oktawa 4
- Z X C V B N M