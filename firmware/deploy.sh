USER=pi
HOST=192.168.100.174
DIR=/home/pi/flashFiles
docker compose up

scp build/Firmware/wombat.bin $USER@$HOST:$DIR/wombat.bin || exit 1
ssh $USER@$HOST "cd $DIR && bash ./wallaby_flash"

docker compose down
 