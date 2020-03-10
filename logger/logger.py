import sys
try:
    try:
        import uio as io
    except ImportError:
        import io
except ImportError:
    print("SKIP")
    raise SystemExit

class Logger:
    def __init__(self, log_file):
        self.log_file = open(log_file, 'w')

        if hasattr(sys, 'print_exception'):
            self.print_exc = sys.print_exception
        else:
            import traceback
            self.print_exc = lambda e, f: traceback.print_exception(None, e, sys.exc_info()[2], file=f)

    def print_exception(self, exception):
        buf = io.StringIO()
        self.print_exc(exception, buf)
        s = buf.getvalue()
        for l in s.split("\n"):
            if l.startswith("  File "):
                l = l.split('"')
                print(l[0], l[2], file=self.log_file)
            elif not l.startswith("    "):
                print(l, file=self.log_file)


if __name__ == "__main__":
    def main():
        raise Exception('msg')
        #raise ZeroDivisionError

    # basic exception message
    try:
        logger = Logger('logs.txt')
        main()
    except Exception as e:
        logger.print_exception(e)
