from flask import Flask, send_from_directory, request
import os
import time
import sys

opStatusFilePath = sys.argv[1]
gcodeFilePath = sys.argv[2]

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
    with open(opStatusFilePath, 'r+') as opStatusFile:
        status = opStatusFile.read()
        print(status)
        if status == "waiting":
            # write gcodefile
            with open(gcodeFilePath, 'r+') as gcodeFile:
                gcodeFile.write
            opStatusFile.write("queued")
            return "Plotting request accepted", 202
            # return job accepted
        else:
            # another job is already queued. return conflict
            return "Conflict. Job is already running", 409

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
