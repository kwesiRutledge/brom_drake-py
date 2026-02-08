
install-as-editable:
# Update the version of the package in setup.py
	sed -i '' "s/{{VERSION_PLACEHOLDER}}/0.0.0+localDevelopment/g" setup.py
# Install the package in editable mode
	pip install -e .
# Restore the placeholder
	sed -i '' "s/0.0.0+localDevelopment/{{VERSION_PLACEHOLDER}}/g" setup.py

lint:
	black .