import asyncio
import time
from typing import Any, Callable, Union, Dict, Tuple

import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
from libstp.device import NativeDevice

from libstp_helpers.api.steps import Step
from libstp_helpers.utility.logging import log


class DataPlotterStep(Step):
    """
    A step that collects data from a function, plots it, and saves it to a CSV file.
    """

    def __init__(self, duration_s: float, data_func: Callable[..., Union[Dict, Tuple, Any]], filename: str):
        """
        Initialize a DataPlotterStep.

        Args:
            duration_s: The duration in seconds to collect data.
            data_func: The function to call to get data points.
            filename: The name of the file to save the data and plot.
        """
        super().__init__()
        self.duration_s = duration_s
        self.data_func = data_func
        self.filename = filename

    async def run_step(self, device: NativeDevice, definitions: Any) -> None:
        """
        Execute the data collection and plotting.

        Args:
            device: The device to run on.
            definitions: Additional definitions needed for execution.
        """
        await super().run_step(device, definitions)

        data = []
        start_time = time.time()

        while time.time() - start_time < self.duration_s:
            if asyncio.iscoroutinefunction(self.data_func):
                result = await self.data_func(device, definitions)
            else:
                result = self.data_func(device, definitions)
            data.append(result)
            await asyncio.sleep(0.01)  # Small delay to prevent busy-waiting

        df = pd.DataFrame(data)
        
        # Save data to CSV
        df.to_csv(f"{self.filename}.csv", index=False)

        # Plot data using seaborn
        plot = sns.relplot(data=df, kind="line")
        plot.savefig(f"{self.filename}.png")
        plt.show()


@log
def data_plotter(
        duration_s: float,
        data_func: Callable[..., Union[Dict, Tuple, Any]],
        filename: str) -> DataPlotterStep:
    """Create a DataPlotterStep."""
    return DataPlotterStep(duration_s=duration_s, data_func=data_func, filename=filename)
