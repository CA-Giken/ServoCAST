# Servo CAST  
The open source technology for the control of Bait Casting Reels

## The Goals  

1. Compatible with the [Arduino](https://www.arduino.cc/)  

2. Servo technology to control the line tension or slack  

3. Minimal Circuit  

4. Monitoring and Setting tools  

5. Technology disclosure

## Developper Tips

### with ES(Nordic nRF52840)
#### FS_Nano33BLEパッチ


#### Fast bootパッチ  
nRF52のデフォルトのブートシーケンスは、LPTICK(外部の低速Xtal)の安定を待つため、0.3秒程度かかる。バッテリレスモードではこの遅れが致命的であるが、USTICKのみを使うことでブート時間を短縮できる。



### MP(Renesas RA4M1)
#### Arduino IDE
[参照](https://support.arduino.cc/hc/en-us/articles/9005041052444-Fix-udev-rules-on-Linux#renesas)

1. ダウンロード  
https://github.com/arduino/ArduinoCore-renesas/tree/mainから[post_install.sh](https://github.com/arduino/ArduinoCore-renesas/blob/main/post_install.sh)をダウンロード

2. 実行パーミッションを付与して実行
~~~
chmod +x post_install.sh
sudo ./post_install.sh
~~~

3. 確認  
/etc/udev/rules.d/に??-arduino-renesas.rulesが出来ていればOK
~~~
ls /etc/udev/rules.d
~~~

#### Flush with RFP  
https://github.com/arduino/ArduinoCore-renesas/tree/1.3.1/bootloaders/UNO_R4
~~~
rfp-cli -device ra -port $portname -p dfu_minima.hex
~~~
https://zenn.dev/ichirowo/articles/6aa1614e102bce

#### Skipping the bootloader  
https://forum.arduino.cc/t/r4-without-bootloader/1286274
ブートローダソース
https://github.com/arduino/arduino-renesas-bootloader
