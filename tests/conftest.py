def pytest_addoption(parser):
    parser.addoption("--run-benchmark", action="store_true", help="run benchmark tests")
