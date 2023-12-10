mkdir tmp -p
touch tmp/0.png
rm tmp/*.png
python3 animation.py
mkdir output -p
ffmpeg -framerate 20 -i tmp/%04d.png -c:v libx264 -pix_fmt yuv420p output/route.mp4 -y
