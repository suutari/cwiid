BASEDIR=/home/rkuennem/wiimote/trunk
CWIIDDIR=${BASEDIR}/cwiid/libcwiid

all: wiimote.mexglx

clean:
	rm wiimote.mexglx

wiimote.mexglx: wiimote.cpp wiimote-utils.cpp
	g++ -c  -I/usr/lib/matlab-7.7/extern/include -I/usr/lib/matlab-7.7/simulink/include -DMATLAB_MEX_FILE -ansi -D_GNU_SOURCE -fPIC -pthread -m32  -fexceptions -D_FILE_OFFSET_BITS=64 -I${CWIIDDIR} -Wall   -DMX_COMPAT_32 -O -DNDEBUG  "wiimote.cpp"
	g++ -c  -I/usr/lib/matlab-7.7/extern/include -I/usr/lib/matlab-7.7/simulink/include -DMATLAB_MEX_FILE -ansi -D_GNU_SOURCE -fPIC -pthread -m32  -fexceptions -D_FILE_OFFSET_BITS=64 -I${CWIIDDIR} -Wall  -DMX_COMPAT_32 -O -DNDEBUG  "/usr/lib/matlab-7.7/extern/src/mexversion.c"
	g++ -O -pthread -shared -m32 -Wl,--version-script,/usr/lib/matlab-7.7/extern/lib/glnx86/mexFunction.map -Wl,--no-undefined -w -L${CWIIDDIR} -static -lcwiid -dynamic -o  "wiimote.mexglx"  wiimote.o mexversion.o  -Wl,-rpath-link,/usr/lib/matlab-7.7/bin/glnx86 -L/usr/lib/matlab-7.7/bin/glnx86 -lmx -lmex -lmat -lm -lstdc++
