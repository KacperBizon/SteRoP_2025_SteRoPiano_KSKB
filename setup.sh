#!/bin/bash

echo "STARTING STEROPIANO CONFIGURATION..."
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"


# instalacja VMPK i QSynth jesli nie ma
echo "[1/4] Checking required programs..."

if dpkg -s qsynth >/dev/null 2>&1 && dpkg -s vmpk >/dev/null 2>&1; then
    echo "   -> Qsynth i VMPK are installed. Skipping installation."
else
    echo "   -> Missing programs. Installing..."
    sudo apt-get update
    sudo apt-get install -y qsynth vmpk fluid-soundfont-gm
fi


# lokalizowanie skryptu do odpalania API
echo "[2/4] Looking for boot script..."

TARGET_SCRIPT="$SCRIPT_DIR/boot_api.sh" # powinien byc w tym samym folderze co setup

# jesli nie ma, to blad
if [ ! -f "$TARGET_SCRIPT" ]; then
    echo "CRITICAL ERROR: File'boot_api.sh' not found!"
    echo "Check if 'boot_api.sh' is located in the same folder as this script."
    exit 1
else
    echo "   -> Found in: $TARGET_SCRIPT"
fi


# nadawanie uprawnien wykonywalnosci skryptowi
echo "[3/4] Granting permission to execute..."
chmod +x "$TARGET_SCRIPT"
echo "   -> OK."


# ustawianie skrotu F12
if command -v gsettings &> /dev/null; then
    echo "[4/4] Adding F12 keyboard shortcut..."
    
    NAME="Start SteRoPiano"
    CMD="$TARGET_SCRIPT"
    BINDING="F12"
    PATH_CUSTOM="org.gnome.settings-daemon.plugins.media-keys.custom-keybinding"
    PATH_MKEY="org.gnome.settings-daemon.plugins.media-keys"
    KEY_PATH="/org/gnome/settings-daemon/plugins/media-keys/custom-keybindings/steropiano/"

    # pobranie aktualnej listy skrotow
    CURRENT_LIST=$(gsettings get $PATH_MKEY custom-keybindings)
    
    # sprawdzenie czy juz jest na liscie
    if [[ $CURRENT_LIST == *"$KEY_PATH"* ]]; then
        echo "   -> Shortcut already on the list. Updating path..."
    else
        if [[ "$CURRENT_LIST" == "@as []" ]] || [[ "$CURRENT_LIST" == "[]" ]]; then
            NEW_LIST="['$KEY_PATH']"
        else
            CLEAN_LIST="${CURRENT_LIST//@as /}"
            NEW_LIST="${CLEAN_LIST%]}, '$KEY_PATH']"
        fi
        
        gsettings set $PATH_MKEY custom-keybindings "$NEW_LIST"
    fi

    # ustawienie parametrow skrotu
    gsettings set $PATH_CUSTOM:$KEY_PATH name "$NAME"
    gsettings set $PATH_CUSTOM:$KEY_PATH command "$CMD"
    gsettings set $PATH_CUSTOM:$KEY_PATH binding "$BINDING"
    
    echo "   -> SUCCESS! F12 shortcut configured."
else
    echo "WARNING: Mission failed."
    echo "You need to configure shortcut by yourself: $TARGET_SCRIPT"
fi

echo "DONE!"
