FROM python:3.12

WORKDIR /app

COPY requirements.txt ./
COPY Car.py ./
COPY Perception.py ./
COPY Sensors.py ./
COPY simulation.py ./
COPY config.yaml ./
COPY ControlMethods.py ./

RUN pip install --no-cache-dir -r requirements.txt

RUN apt update && apt install -y \
    python3-pygame \
    x11-apps \
    && pip install pygame

CMD ["python", "-u", "simulation.py", "config.yaml"]