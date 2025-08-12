import asyncio

from libstp.asynchronous import sample_algorithm
from libstp_helpers.utility import to_task


async def main():
    print("Items")
    task1 = to_task(sample_algorithm())
    await asyncio.sleep(0.2)
    task2 = to_task(sample_algorithm())
    await task1
    await task2


if __name__ == "__main__":
    asyncio.run(main())
