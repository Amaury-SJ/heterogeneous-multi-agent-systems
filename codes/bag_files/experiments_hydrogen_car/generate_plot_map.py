import pandas as pd
import plotly.express as px

# Load text file and read data
file_name = "GNSSM2_raw_20240325143439.pos"  # Replace this with the path of your file
with open(file_name, 'r') as file:
    lines = file.readlines()

# Pre-process data to separate it into columns
data = [line.strip().split() for line in lines]

# Create a pandas DataFrame using all available columns
df = pd.DataFrame(data, columns=['date', 'heure', 'latitude', 'longitude', 'colonne_5', 'colonne_6',
                                 'colonne_7', 'colonne_8', 'colonne_9', 'colonne_10', 'colonne_11',
                                 'colonne_12', 'colonne_13', 'colonne_14', 'colonne_15'])

# Select the columns needed to form the final DataFrame
df = df[['date', 'heure', 'latitude', 'longitude']]

# Convert relevant columns to numeric types if necessary
df['latitude'] = pd.to_numeric(df['latitude'][10:])
df['longitude'] = pd.to_numeric(df['longitude'][10:])

# Creating a Plotly figure
fig = px.line_mapbox(df, lat='latitude', lon='longitude', hover_name='date', zoom=12, ) # Other arg: height=2000

# Mark the starting point
fig.add_trace(px.scatter_mapbox(df.head(1), lat='latitude', lon='longitude', hover_name='date').data[0])

fig.update_layout(mapbox_style="open-street-map")

# Save map as HTML file
fig.write_html("plot_track_map.html")
