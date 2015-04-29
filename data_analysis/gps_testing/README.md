filtered_pos.py provides a record type for the information recorded on the /filtered_pos ROS topic


gps_data.py provides functions to load CSV files with /filtered_pos data (using the FilteredPos record type mentioned previously), extract lists of GPS coordinates, and extract the time each GPS point was recorded relative to the beginning of the file.


plotting.py provides two utility functions for working with the Python plotting library [matplotlib](matplotlib.org).

To convert a given topic (TOPIC) recorded in a bagfile (FILE) to a CSV file, run the following command. 

```bash
rostopic echo -b FILE -p /TOPIC
```
