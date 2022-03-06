readme="./images/README.md"
for file in ./images/*
do
    if [ "$file" != "$readme" ]; then
        echo $file
        edge-impulse-uploader $file
    fi 
done
