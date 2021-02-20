
import sys
import math

if len(sys.argv) < 2:
    raise Exception("no normal file supplied")
path = sys.argv[1]
print(f"getting normals from file at \"{path}\"")


def mag(vec):
    magsq = 0
    for i in range(3):
        magsq += vec[i] * vec[i]
    return math.sqrt(magsq)


def normalize(vec):
    norm = mag(vec)
    if norm <= 0:
        raise Exception("cannot normalize null vector")

    for i in range(3):
        vec[i] = vec[i] / norm

    return vec


def quantize(vec, to):
    for i in range(3):
        vec[i] = round(vec[i] * to)


def sub(a, b):
    out = [0, 0, 0]
    for i in range(3):
        out[i] = a[i] - b[i]
    return out


databuf = []
generated = []


def gen_normal(databuf):
    total = [0, 0, 0]
    total_mag = 0
    for normal in databuf:
        total_mag += mag(normal)
        normalize(normal)
        for i in range(3):
            total[i] += normal[i]
    total_mag /= len(databuf)

    normalize(total)
    quantize(total, 1024)
    dist_to_home = 0
    if len(generated) > 0:
        dist_to_home = mag(sub(total, generated[0]))
    generated.append(total)
    print(
        f"{{{total[0]}, {total[1]}, {total[2]}}}, // Average magnitude {total_mag}{'' if dist_to_home == 0 else f' ({round(dist_to_home)} from home)'}")


file = open(path, "r")
for line in file.readlines():
    line = line.strip()
    if line == "":
        if len(databuf) > 0:
            gen_normal(databuf)
            databuf = []
    else:
        normal = list(map(lambda n: n.strip(), line.split(",")))
        if len(normal) != 3:
            raise Exception(
                f"line '{line}': expected 3 elements, found {len(normal)}")
        for i in range(3):
            asint = int(normal[i])
            if asint == None:
                raise Exception(
                    f"line '{line}': element {i} ({normal[i]}) is not an int")
            else:
                normal[i] = asint
        databuf.append(normal)
file.close()

if len(databuf) > 0:
    gen_normal(databuf)
    databuf = []
