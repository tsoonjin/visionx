#!/usr/bin/zsh
echo "Synching utils"
rsync -avzr --exclude 'bag' /home/papa/Project/bbauv/bbauv/src/vision/script/utils  bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
if [ $1 = "all" ];then
	echo "Synching return home"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/home bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
	echo "Synching lane"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/lane bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
	echo "Synching bins"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/bins bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
fi
if [ $1 = "home" ];then
	echo "Synching return home"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/home bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
fi
if [ $1 = "lane" ];then
	echo "Synching lane"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/lane bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
fi
if [ $1 = "bins" ];then
	echo "Synching bins"
	rsync -avzr /home/papa/Project/bbauv/bbauv/src/vision/script/bins bbauvsbc1@bbauv:~/bbauv/src/vision/scripts --progress 
fi

