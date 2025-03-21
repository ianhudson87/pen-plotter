from flask import Flask, send_from_directory
import os

app = Flask(__name__)

# Define the path to the directory containing the HTML file
HTML_DIR = os.path.abspath(".")  # Current directory

@app.route('/')
def serve_html():
    return send_from_directory(HTML_DIR, "index.html")

@app.route('/plot')
def serve_html():
    print("Plotting!")

if __name__ == '__main__':
    app.run(debug=True, host='0.0.0.0', port=5000)
