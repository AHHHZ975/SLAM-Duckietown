from flask import Flask, request, render_template_string
import os

app = Flask(__name__)

# Directory to save uploaded images
UPLOAD_FOLDER = 'uploads'
os.makedirs(UPLOAD_FOLDER, exist_ok=True)

@app.route('/')
def index():
    # Display the most recently uploaded image, if available
    images = os.listdir(UPLOAD_FOLDER)
    if images:
        latest_image = os.path.join(UPLOAD_FOLDER, sorted(images)[-1])
        return render_template_string("""
            <h1>Uploaded Image</h1>
            <img src="/{{ image_path }}" alt="Uploaded Image" style="max-width:100%; height:auto;">
        """, image_path=latest_image)
    else:
        return "<h1>No images uploaded yet!</h1>"

@app.route('/upload', methods=['POST'])
def upload_image():
    # Check if an image is uploaded
    if 'image' not in request.files:
        return "No image file uploaded", 400

    file = request.files['image']
    if file.filename == '':
        return "No selected file", 400

    # Save the uploaded image
    image_path = os.path.join(UPLOAD_FOLDER, file.filename)
    file.save(image_path)

    return f"Image {file.filename} uploaded successfully!"

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000)
