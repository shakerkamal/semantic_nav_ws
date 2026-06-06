from semantic_nav_semantics.llama_action_client import LlamaActionClient


class _Result:
    def __init__(self, text):
        self.response = type("R", (), {"text": text})()


class _Wrap:
    STATUS_SUCCEEDED = 4

    def __init__(self, text):
        self.status = self.STATUS_SUCCEEDED
        self.result = _Result(text)


class _GoalHandle:
    accepted = True

    def __init__(self, text):
        self._text = text

    def get_result_async(self):
        class _F:
            done_called = False

            def __init__(self, text):
                self._text = text

            def done(self):
                self.done_called = True
                return True

            def result(self):
                return _Wrap(self._text)

            def exception(self):
                return None
        return _F(self._text)


class _Future:
    def __init__(self, gh):
        self._gh = gh
        self._done = False

    def done(self):
        self._done = True
        return True

    def result(self):
        return self._gh

    def exception(self):
        return None


class _FakeAction:
    def __init__(self, text="hello"):
        self._text = text

    def wait_for_server(self, timeout_sec):
        return True

    def send_goal_async(self, goal):
        return _Future(_GoalHandle(self._text))


class _NullLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def test_call_returns_text():
    fake = _FakeAction("{\"x\":1}")
    client = LlamaActionClient(action_client=fake, logger=_NullLogger())
    text = client.call(prompt="hi", gbnf_grammar="root ::= [a-z]+", max_tokens=8)
    assert text == "{\"x\":1}"
