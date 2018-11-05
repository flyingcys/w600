@rem 参数0: exe
@rem 参数1: 输入bin文件 ,原始文件或者压缩档文件
@rem 参数2: 输出文件(目标生成文件）
@rem 参数3: 输入文件类型,0是image文件，1是FLASHBOOT文件 2是secboot文件
@rem 参数4: 是否压缩文件：0：plain文件，1：压缩类型文件
@rem 参数5: 版本号文件
@rem 参数6：升级文件再FLASH里的存放位置（相对位置）
@rem 参数7：升级后的文件启动位置（相对位置）
@rem 参数8：原始bin文件

@echo off

copy build\rtthread-w600.map Bin
copy rtthread.bin Bin
cd Libraries

copy rtthread\version.txt ..\Bin
copy rtthread\secboot.img ..\Bin

cd Tools

wm_gzip.exe "..\..\Bin\rtthread.bin"
makeimg.exe "..\..\Bin\rtthread.bin" "..\..\Bin\rtthread.img" 0 0 "..\..\Bin\version.txt" 90000 10100
makeimg.exe "..\..\Bin\rtthread.bin.gz" "..\..\Bin\rtthread_GZ.img" 0 1 "..\..\Bin\version.txt" 90000 10100 "..\..\Bin\rtthread.bin" 
makeimg.exe "..\..\Bin\rtthread.bin" "..\..\Bin\rtthread_SEC.img" 0 0 "..\..\Bin\version.txt" 90000 10100
makeimg_all.exe "..\..\Bin\secboot.img" "..\..\Bin\rtthread.img" "..\..\Bin\rtthread.FLS"
@del "..\..\Bin\rtthread.img"


