while [ 1 ]
do
        date
        top -n 1 | grep mainapp | grep -v grep
        sleep 30
done

