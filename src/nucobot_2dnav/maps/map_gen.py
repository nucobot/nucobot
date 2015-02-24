#!/usr/bin/env python
import sys
import argparse

parser = argparse.ArgumentParser(description='PGM map generator')
parser.add_argument("filename", help="Output file name")
parser.add_argument("mul", help="Size multiplyer", type=float)
args = parser.parse_args() 

size_mul = args.mul

line_size = int(22*size_mul)

def rectangle(map, startx, endx, starty, endy):
	for i in range(starty, endy):
		for j in range(startx, endx):
				map[i][j] = chr(0)


if len(sys.argv) < 2:
	exit()

f = open(args.filename, 'w')

size = [int(2000*size_mul), int(3000*size_mul)]


#File header
f.write('P5\n')
f.write(str(size[0])+' '+str(size[1])+'\n')
f.write('255\n')

#Generating map
map = [[chr(255) for x in range(size[0])] for x in range(size[1])] 

#Borders

#Left border
rectangle(map, 0, line_size, 0, size[1])
#right border
rectangle(map, size[0]-line_size, size[0], 0, size[1])
#top border
rectangle(map, 0, size[0], 0, line_size)
#bottom border
rectangle(map, 0, size[0], size[1]-line_size, size[1])

#Houses

#Top
#left line
rectangle(map, int(800*size_mul)-line_size, int(800*size_mul), 0, int(400*size_mul))
#right line
rectangle(map, int(1200*size_mul), int(1200*size_mul)+line_size, 0, int(400*size_mul))
#horisontal line 70 is it top
rectangle(map, int(800*size_mul)-line_size, int(1200*size_mul)+line_size, int(70*size_mul)-line_size, int(70*size_mul))

#Bottom
#left line
rectangle(map, int(800*size_mul)-line_size, int(800*size_mul), int(2600*size_mul), size[1])
#right line
rectangle(map, int(1200*size_mul), int(1200*size_mul)+line_size, int(2600*size_mul), size[1])
#horisontal line 70 is it top
rectangle(map, int(800*size_mul)-line_size, int(1200*size_mul)+line_size, int(2930*size_mul), int(2930*size_mul)+line_size)


#Stairs
#TopLine
rectangle(map, 0, int(580*size_mul), int(967*size_mul), int(967*size_mul)+line_size)
#BottomLine
rectangle(map, 0, int(580*size_mul), int(2033*size_mul), int(2033*size_mul)+line_size)
#MiddleLine
rectangle(map, 0, int(580*size_mul), int(1506*size_mul), int(1506*size_mul)+line_size)


#Platform
rectangle(map, int(1900*size_mul), size[0], int(1200*size_mul), int(1800*size_mul))


#Saving to file
for i in range(0, size[1]):
	for j in range(0, size[0]):
		if j == size[0] - 1:
			f.write(map[i][j])
		else:
			f.write(map[i][j])



