from __future__ import annotations

import os.path


class Logger:
    def __init__(
        self,
        headers: list[str],
        filename: str
    ) -> None:
        self.headers = headers
        self.filename = filename

        dir =  f"{os.path.expanduser('~')}/.ros/log/sgaar"

        if not os.path.exists(dir):
            os.makedirs(dir)  
        self.file = open(
            f"{dir}/{self.filename}",
            "w+")
        
        self.log(self.headers)

    def log(self, data: list) -> None:
        self.file.write(f"{','.join([str(d) for d in data])}\n")

    def close(self) -> None:
        self.file.close()
