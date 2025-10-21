
docker pull ubuntu:20.04

docker run -it \
--gpus all \
-v "/home/videopipe:/home/videopipe" \
--rm ubuntu:20.04


sh -c 'printf " \
deb http://archive.ubuntu.com/ubuntu/ focal main restricted\n \
deb http://archive.ubuntu.com/ubuntu/ focal-updates main restricted\n \
deb http://archive.ubuntu.com/ubuntu/ focal universe\n \
deb http://archive.ubuntu.com/ubuntu/ focal multiverse\n \
deb http://archive.ubuntu.com/ubuntu/ focal-updates multiverse\n \
deb http://archive.ubuntu.com/ubuntu/ focal-backports main restricted universe multiverse\n \
deb http://security.ubuntu.com/ubuntu/ focal-security main restricted\n \
deb http://security.ubuntu.com/ubuntu/ focal-security universe\n \
deb http://security.ubuntu.com/ubuntu/ focal-security multiverse" \
> /etc/apt/sources.list'


apt update &&
apt install -y build-essential vim cmake pkg-config python3 gperf

gcc -v

export ROOT_SRC_CODE_PATH=/home/videopipe


apt install -y nasm yasm libx264-dev libx265-dev libvpx-dev libfdk-aac-dev libmp3lame-dev libopus-dev libvorbis-dev libtheora-dev


cd ${ROOT_SRC_CODE_PATH}/openssl-1.0.1f
./config shared \
&& make && make install

报错：POD document had syntax errors at /usr/bin/pod2man line 69.
rm -f /usr/bin/pod2man


vi /etc/profile
export PATH=/usr/local/ssl/bin:$PATH
export C_INCLUDE_PATH=/usr/local/ssl/include:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=/usr/local/ssl/include:$CPLUS_INCLUDE_PATH
export LIBRARY_PATH=/usr/local/ssl/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/ssl/lib:$LD_LIBRARY_PATH
source /etc/profile

openssl version

cd ${ROOT_SRC_CODE_PATH}/zlib-1.2.11
./configure \
&& make && make install


wget https://mirror.souseiseki.middlendian.com/nongnu/freetype/freetype-2.13.2.tar.gz
cd ${ROOT_SRC_CODE_PATH}/freetype-2.13.2/
#rely: zlib
./configure \
&& make && make install


# nv-codec-headers
wget https://github.com/FFmpeg/nv-codec-headers/releases/download/n11.1.5.3/nv-codec-headers-11.1.5.3.tar.gz
cd ${ROOT_SRC_CODE_PATH}/nv-codec-headers-11.1.5.3
make && make install


# cuda 12.2
wget wget https://developer.download.nvidia.com/compute/cuda/12.2.2/local_installers/cuda_12.2.2_535.104.05_linux.run
sh cuda_12.2.2_535.104.05_linux.run
# 不要安装驱动

nvcc -V


# cudnn
wget https://developer.download.nvidia.com/compute/cudnn/secure/8.9.6/local_installers/12.x/cudnn-local-repo-ubuntu2004-8.9.6.50_1.0-1_amd64.deb?__token__=exp=1758704462~hmac=b265073698e6916dd0ec2ecd97a5e43b2823a7c499cde1c2dfbcd9081402c607&t=eyJscyI6InJlZiIsImxzZCI6IlJFRi1ibG9nLmNzZG4ubmV0L3dlaXhpbl80NzE0NTA1NC9hcnRpY2xlL2RldGFpbHMvMTQwNDIyNDAzIn0=
dpkg -i cudnn-local-repo-ubuntu2004-8.9.6.50_1.0-1_amd64.deb
cp /var/cudnn-local-repo-ubuntu2004-8.9.6.50/cudnn-local-5E60450C-keyring.gpg /usr/share/keyrings/
apt-get update
apt install libcudnn8-dev
# 确定cudnn 安装和版本
apt show libcudnn8

# Video_Codec_SDK
wget https://developer.download.nvidia.com/designworks/video-codec-sdk/secure/12.0/Video_Codec_SDK_12.0.16.zip?__token__=exp=1758784256~hmac=a4813475d85daa11a2b7ca09fbf925bfcaf091f4dfa002c8af010e75ee46ff67&t=eyJscyI6InJlZiIsImxzZCI6IlJFRi1ibG9nLmNzZG4ubmV0L3dlaXhpbl80NzE0NTA1NC9hcnRpY2xlL2RldGFpbHMvMTQwNDIyNDAzIn0=
cd ${ROOT_SRC_CODE_PATH}
cp ${ROOT_SRC_CODE_PATH}/Video_Codec_SDK_12.0.16/Lib/linux/stubs/x86_64/*  /usr/local/cuda/lib64/stubs && \
cp ${ROOT_SRC_CODE_PATH}/Video_Codec_SDK_12.0.16/Interface/* /usr/local/cuda/include && \
cp ${ROOT_SRC_CODE_PATH}/Video_Codec_SDK_12.0.16/Lib/linux/stubs/x86_64/*  /usr/local/cuda-12.2/lib64/stubs && \
cp ${ROOT_SRC_CODE_PATH}/Video_Codec_SDK_12.0.16/Interface/* /usr/local/cuda-12.2/include && \
cp ${ROOT_SRC_CODE_PATH}/Video_Codec_SDK_12.0.16/Interface/* /usr/include


wget https://github.com/FFmpeg/FFmpeg/archive/n4.2.1.tar.gz
cd ${ROOT_SRC_CODE_PATH}/FFmpeg-n4.2.1

./configure \
--enable-gpl \
--enable-libfdk-aac \
--enable-libfreetype \
--enable-libmp3lame \
--enable-libopus \
--enable-libtheora \
--enable-libvorbis \
--enable-libx264 \
--enable-libx265 \
--enable-nonfree \
--enable-shared \
--disable-static \
--enable-nvenc \
--enable-cuda \
--enable-cuvid

make -j32 && make install

ffmpeg -version


vi /etc/profile
export PATH=/usr/local/ssl/bin:/usr/local/cuda/bin:$PATH
export C_INCLUDE_PATH=/usr/local/ssl/include:/usr/local/include:$C_INCLUDE_PATH
export CPLUS_INCLUDE_PATH=/usr/local/ssl/include:/usr/local/include:$CPLUS_INCLUDE_PATH
export LIBRARY_PATH=/usr/local/ssl/lib:/usr/local/lib:$LIBRARY_PATH
export LD_LIBRARY_PATH=/usr/local/ssl/lib:/usr/local/cuda/lib64:/usr/local/cuda/lib64/stubs:/usr/local/lib:$LD_LIBRARY_PATH
source /etc/profile


apt-get clean

docker commit a0df03febc16 ubuntu:20.04_cuda12.2

docker save \
-o ubuntu20.04_cuda12.2.tar \
ubuntu:20.04_cuda12.2