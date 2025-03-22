from flask import Flask, send_from_directory, request
import os
import time
import sys

app = Flask(__name__)

# Define the path to the directory containing the HTML file
HTML_DIR = os.path.abspath(".")  # Current directory

@app.route('/')
def serve_html():
    return send_from_directory(HTML_DIR, "index.html")

@app.route('/plot', methods=['POST'])
def plot():
    print("plot requested")
    print(request.json)
    with open(opStatusFilePath, 'r') as opStatusFile:
        status = opStatusFile.read()

    print(f"Current status: {status}")

    if status == "waiting":
        with open(gcodeFilePath, 'w') as gcodeFile:
            gcodeFile.write(request.json['gcode'])
        with open(opStatusFilePath, 'w') as opStatusFile:
            opStatusFile.write("queued")
        return "Plotting request accepted", 202
        # return job accepted
    else:
        # another job is already queued. return conflict
        return "Conflict. Job is already running", 409

if __name__ == '__main__':
    if(len(sys.argv) < 3):
        print("Usage: server.py [opStatusFilePath] [gcodeFilePath]")

    opStatusFilePath = sys.argv[1]
    gcodeFilePath = sys.argv[2]

    app.run(debug=True, host='0.0.0.0', port=5000)
