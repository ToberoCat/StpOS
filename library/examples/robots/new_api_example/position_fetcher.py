from external_tracker_client import Configuration, DefaultApi, ApiClient


class PositionFetcher:
    def __init__(self, tracker_api="http://192.168.31.172:8000"):
        self.configuration = Configuration(
            host=tracker_api,
        )
        self.api_client = ApiClient(configuration=self.configuration)
        self.client = DefaultApi(api_client=self.api_client)


    def fetch_next_position(self):
        markers = self.client.get_markers_markers_get()
        if not markers.robot_found:
            return
        return markers.robot, markers.robot_angle