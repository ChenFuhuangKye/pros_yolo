export IMG_NAME="pros_yolo"
export ECR_URL="ocar1054"
export TAG="0.0.0"
export DOCKER_CLI_EXPERIMENTAL=enabled

docker run --rm --privileged tonistiigi/binfmt:latest
docker run --privileged --rm tonistiigi/binfmt --uninstall qemu-*
docker run --privileged --rm tonistiigi/binfmt --install all
docker buildx create --use --platform=linux/arm64,linux/amd64 --name multi-platform-builder
docker buildx inspect --bootstrap
docker buildx build --platform=linux/arm64,linux/amd64 --push \
    --tag $ECR_URL/$IMG_NAME:latest \
    --tag $ECR_URL/$IMG_NAME:$TAG \
    -f ./Dockerfile .