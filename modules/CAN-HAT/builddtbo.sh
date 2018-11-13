#dtoverlay -r mcp2517fd-can0

dtc -@ -I dts -O dtb -o 2xMCP2517FD.dtbo 2xMCP2517FD-overlay.dts

# cp *.dtbo /boot/overlays
# dtoverlay mcp2517fd-can0
