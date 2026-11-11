FROM python:3.12-slim

ENV PYTHONDONTWRITEBYTECODE=1 \
    PYTHONUNBUFFERED=1 \
    MUJOCO_GL=osmesa \
    MUJOCO_LANDING_MODEL=/app/output/landing_controller.joblib

RUN apt-get update \
    && apt-get install -y --no-install-recommends libosmesa6 libgl1 \
    && apt-get clean

WORKDIR /app/src/air/mujoco_uav_moving_pad_landing
COPY requirements.txt ./
RUN python -m pip install --no-cache-dir -r requirements.txt
COPY . ./

CMD ["python", "main.py", "demo", "--steps", "180", "--output", "/app/output"]
