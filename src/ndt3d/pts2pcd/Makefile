# Makefile
CC     = g++
TARGET = pts2pcd
SRCS   = main.cpp
OBJS   = $(SRCS:.cpp=.o)
 
# インクルードファイルのあるディレクトリパス
INCDIR  = -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -I$(HOME)/include
 
# ライブラリファイルのあるディレクトリパス
LIBS    = -lpcl_io -lpcl_common -L/usr/lib -lboost_system -L$(HOME)/lib -lm

# コンパイルオプション等
CFLAGS  = -std=c++11
DEBUG   = -D DEBUG

# リンク時にライブラリを追加
$(TARGET): $(OBJS)
	$(CC) -o $@ $^ $(LIBS)
 
# ソースをコンパイルするときにインクルードディレクトリを指定
$(OBJS): $(SRCS)
	$(CC) $(CFLAGS) $(DEBUG) $(INCDIR) -c $(SRCS)

clean:
	rm -f $(TARGET)
	rm -f *.o *.obj
	rm -f *~ *.~*
