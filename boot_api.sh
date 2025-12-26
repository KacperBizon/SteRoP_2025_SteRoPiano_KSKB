#!/bin/bash

# zapisanie sciezki katalogu projektu
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

# namierzenie plikow do wczytania
ORIGINAL_SESSION="$SCRIPT_DIR/vmpk_settings.vmpk"
REAL_KEYMAP="$SCRIPT_DIR/key_map.xml"

# plik dla aktualnej sesji vmpk
TEMP_SESSION="/tmp/vmpk_session_run.vmpk"

# skopiowanie oryginalnej sesji do temp
cp "$ORIGINAL_SESSION" "$TEMP_SESSION"

# odnalezienie linijki <KeyMapFile> i podmianka sciezki do key_map.xml na aktualna
sed -i "s|<KeyMapFile>.*</KeyMapFile>|<KeyMapFile>$REAL_KEYMAP</KeyMapFile>|g" "$TEMP_SESSION"

# sprawdzenie i ewentualne uruchomienie qsynth
if pgrep -x "qsynth" > /dev/null
then
    echo "Qsynth already running."
else
    echo "Starting Qsynth..."
    qsynth &
    sleep 3
fi

# sprawdzenie i ewentualne uruchomienie vmpk
if pgrep -x "vmpk" > /dev/null
then
    echo "VMPK already running."
else
    echo "Starting and configuring VMPK..."
    vmpk "$TEMP_SESSION" & # uruchomienie ze wskazanym plikiem z tmp
    sleep 2
fi

# polaczenie
aconnect "VMPK Output" "FLUID Synth" 2>/dev/null

echo "Ready! Used keyboard map: $REAL_KEYMAP"
