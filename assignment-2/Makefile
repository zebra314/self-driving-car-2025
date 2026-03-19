all: build run

build:
	docker compose build

run:
	xhost +local:root
	docker compose run --rm ros1_dev bash
	xhost -local:root

attach:
	docker compose exec ros1_dev bash

clean:
	docker compose down -v --rmi local
