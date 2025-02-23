package frc.robot.subsystems.canWatchdog;

import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import java.io.IOException;
import java.net.URI;
import java.net.http.HttpClient;
import java.net.http.HttpRequest;
import java.net.http.HttpResponse;
import java.util.HashSet;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class CANWatchdogIOComp implements CANWatchdogIO {
  private final ObjectReader reader =
      new ObjectMapper()
          .configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false)
          .reader();

  @Override
  public Stream<Integer> getIds(String jsonBody) {
    try {
      return reader.readTree(jsonBody).findPath("DeviceArray").findValues("ID").stream()
          .map(JsonNode::numberValue)
          .map(Number::intValue);
    } catch (IOException e) {
      // e.printStackTrace();
      return Stream.empty();
    }
  }

  @Override
  public int[] checkForMissingIds() {

    final URI uri = URI.create(CANWatchdogConstants.REQUEST);
    final HttpClient client = HttpClient.newHttpClient();
    try {
      // open a tcp socket to phoenix tuner to get the list of devices
      var body =
          client
              .send(
                  HttpRequest.newBuilder().uri(uri).GET().build(),
                  HttpResponse.BodyHandlers.ofString())
              .body();

      if (body == null) return new int[0];
      ;

      // parse the json and check if all the ids we expect are present
      Set<Integer> ids = getIds(body).collect(Collectors.toCollection(HashSet::new));
      if (ids.containsAll(CANWatchdogConstants.CAN.getIds())) {
        return new int[0];
      } else {
        return CANWatchdogConstants.CAN.getDevices().stream()
            .filter(device -> !(ids.contains(device.id())))
            .map(device -> device.id())
            .mapToInt(id -> id)
            .toArray();
      }
    } catch (InterruptedException e) {
      // restore the interrupted status
      Thread.currentThread().interrupt();
    } catch (IOException e) {
      e.printStackTrace();
    }
    return new int[] {-1};
  }
}
