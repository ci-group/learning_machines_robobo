FROM python:3.8.19-bookworm

WORKDIR /app
RUN python -m venv /venv

COPY ./patch_requirements.txt ./patch_requirements.txt
RUN /venv/bin/python -m pip install -r ./patch_requirements.txt

COPY ./test.py ./test.py

CMD ["/venv/bin/python", "./test.py"]
