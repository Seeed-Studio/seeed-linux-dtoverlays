#dtoverlay -r mcp25xxfd

dtc -@ -I dts -O dtb -o 2xMCP2517FD.dtbo 2xMCP2517FD-overlay.dts
dtc -@ -I dts -O dtb -o 2xMCP2518FD-spi0.dtbo 2xMCP2518FD-spi0-overlay.dts

# cp *.dtbo /boot/overlays
# dtoverlay mcp25xxfd
