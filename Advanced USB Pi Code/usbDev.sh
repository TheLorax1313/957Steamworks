# Navigates to the HOME folder and resets a data folder for USB information
cd /home/pi
rm -rf usbData
mkdir usbData
cd /home/pi/usbData

# Finds serial numbers of all cameras in the /dev/videoX directories
/bin/udevadm info --name=/dev/video0 > tempData.txt
sed -e '1,16d;18,33d' tempData.txt >> usbData.txt

/bin/udevadm info --name=/dev/video1 > tempData.txt
sed -e '1,16d;18,33d' tempData.txt >> usbData.txt

/bin/udevadm info --name=/dev/video2 > tempData.txt
sed -e '1,16d;18,33d' tempData.txt >> usbData.txt

# Cleans up the data and deletes extra files.
sed -r 's/.{19}//' usbData.txt > usbDataFinal.txt
rm usbData.txt
rm tempData.txt