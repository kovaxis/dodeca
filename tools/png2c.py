from PIL import Image
import sys
import os
import math

if len(sys.argv) < 2:
    raise Exception("expected font directory path / input file")

if os.path.isdir(sys.argv[1]):
    path = sys.argv[1]
    files = list(os.listdir(path))
    files.sort()
    name = path
    make_r90 = True
else:
    path = '.'      # For a single input file, run this script from the folder containing input
    files = (sys.argv[1],)
    name = os.path.splitext(sys.argv[1])[0]
    make_r90 = False


def make_font(images, flavor):
    for image in images:
        print(" ", end="")
        bytebuf = 0
        bytebuf_curbit = 0
        for x in range(image.width):
            for y in range(image.height):
                pix = any(image.getpixel((x, y)))
                if pix:
                    bytebuf |= 1 << bytebuf_curbit
                bytebuf_curbit += 1
                if bytebuf_curbit >= 8:
                    print(f" {bytebuf :#04x},", end="")
                    bytebuf = 0
                    bytebuf_curbit = 0
        if bytebuf_curbit > 0:
            print(f" {bytebuf :#04x},", end="")
        print()


images = []
for entry in files:
    if entry[-4:] == ".png":
        image = Image.open(path+"/"+entry)
        images.append(image)
assert all(map(lambda img: img.width ==
               images[0].width and img.height == images[0].height, images))

print(
    f"// Font \"{name}\" ({images[0].width}x{images[0].height})")
print(f"const PROGMEM byte FONT_{name}_DATA[] = {{")

make_font(images, "DATA")

if make_r90:
    for i in range(len(images)):
        images[i] = images[i].transpose(Image.ROTATE_270)
    make_font(images, "DATA_R90")

print("};")
