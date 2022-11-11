from __future__ import annotations

class Logger:
    def __init__(
        self,
        headers: list[str],
        filename: str
    ) -> None:
        self.headers = headers
        self.filename = filename

        self.file = open(
            f"/home/thomas/.ros/log/seagraves_unmanned_systems_pkg/{self.filename}", 
            "w+")
        self.log(self.headers)

    def log(self, data: list) -> None:
        self.file.write(f"{','.join([str(d) for d in data])}\n")

    def close(self) -> None:
        self.file.close()
