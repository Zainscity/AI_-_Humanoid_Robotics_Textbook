install:
	pip install -r requirements.txt

init-db:
	alembic init migrations

migrate:
	alembic revision --autogenerate -m "$(MESSAGE)"
	alembic upgrade head

run:
	uvicorn src.main:app --reload

test:
	pytest

lint:
	flake8 src/
	black src/ --check
	isort src/ --check-only

format:
	black src/
	isort src/

clean:
	find . -name "__pycache__" -exec rm -rf {} +
	find . -name "*.pyc" -exec rm -f {} +
	find . -name "*.log" -exec rm -f {} +
	rm -rf .pytest_cache .mypy_cache .venv
