#dtoverlay -r seeed-2mic-voicecard
DTC_FLAGS="-b 0 -Wno-unit_address_vs_reg -I dts -O dtb"

dtc -@ $DTC_FLAGS -o seeed-2mic-voicecard.dtbo seeed-2mic-voicecard-overlay.dts

# cp *.dtbo /boot/overlays
# dtoverlay seeed-2mic-voicecard
