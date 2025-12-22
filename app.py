from backend import app

# This file is specifically for Hugging Face Spaces deployment
# It imports and exposes the FastAPI app created in backend.py
# Hugging Face Spaces looks for an 'app' variable to run

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)