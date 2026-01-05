#!/usr/bin/env python3
"""
URC 2026 Terrain ML Analyzer - scikit-learn + TensorFlow Implementation

Professional machine learning-based terrain analysis for Mars rover navigation.
Uses scikit-learn for terrain classification and TensorFlow for advanced pattern
recognition. Replaces manual terrain analysis with trained ML models.

Author: URC 2026 ML Terrain Analysis Team
"""

import numpy as np
import time
from typing import Dict, List, Any, Optional, Tuple
import pickle
import os

# Import ML libraries with fallbacks
try:
    from sklearn.ensemble import RandomForestClassifier, GradientBoostingClassifier
    from sklearn.svm import SVC
    from sklearn.model_selection import train_test_split, cross_val_score
    from sklearn.preprocessing import StandardScaler
    from sklearn.metrics import accuracy_score, classification_report
    from sklearn.pipeline import Pipeline
    import joblib
    SKLEARN_AVAILABLE = True
except ImportError:
    SKLEARN_AVAILABLE = False

try:
    import tensorflow as tf
    from tensorflow import keras
    from tensorflow.keras import layers
    TENSORFLOW_AVAILABLE = True
except ImportError:
    TENSORFLOW_AVAILABLE = False

logger = None  # Will be set when imported


class TerrainFeatureExtractor:
    """Extract features from terrain images for ML classification."""

    def __init__(self):
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn not available for terrain ML analysis")

    def extract_features(self, image: np.ndarray) -> np.ndarray:
        """Extract comprehensive features from terrain image."""
        features = []

        # Color-based features (HSV space)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV) if 'cv2' in globals() else image

        # Statistical features
        for channel in range(3):
            channel_data = hsv[:, :, channel].flatten()
            features.extend([
                np.mean(channel_data),
                np.std(channel_data),
                np.min(channel_data),
                np.max(channel_data),
                np.median(channel_data),
                np.percentile(channel_data, 25),
                np.percentile(channel_data, 75)
            ])

        # Texture features using GLCM-like approach
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if 'cv2' in globals() else image[:, :, 0]
        texture_features = self._extract_texture_features(gray)
        features.extend(texture_features)

        # Edge features
        edges = cv2.Canny(gray, 50, 150) if 'cv2' in globals() else np.zeros_like(gray)
        features.extend([
            np.mean(edges),
            np.std(edges),
            np.sum(edges > 0) / edges.size  # Edge density
        ])

        return np.array(features)

    def _extract_texture_features(self, gray_image: np.ndarray) -> List[float]:
        """Extract texture features from grayscale image."""
        features = []

        # Simple texture metrics
        laplacian = cv2.Laplacian(gray_image, cv2.CV_64F) if 'cv2' in globals() else np.zeros_like(gray_image)
        sobel_x = cv2.Sobel(gray_image, cv2.CV_64F, 1, 0) if 'cv2' in globals() else np.zeros_like(gray_image)
        sobel_y = cv2.Sobel(gray_image, cv2.CV_64F, 0, 1) if 'cv2' in globals() else np.zeros_like(gray_image)

        features.extend([
            np.var(laplacian),  # Texture variance
            np.mean(np.abs(sobel_x)),  # Horizontal gradients
            np.mean(np.abs(sobel_y)),  # Vertical gradients
            np.mean(np.sqrt(sobel_x**2 + sobel_y**2))  # Gradient magnitude
        ])

        return features


class TerrainMLClassifier:
    """
    Machine Learning-based terrain classifier using scikit-learn.

    Trains and uses ML models to classify terrain types with higher accuracy
    than rule-based approaches. Supports multiple algorithms and model persistence.
    """

    def __init__(self, model_path: str = "models/terrain_classifier.pkl"):
        if not SKLEARN_AVAILABLE:
            raise ImportError("scikit-learn required for ML terrain classification")

        self.model_path = model_path
        self.model = None
        self.scaler = StandardScaler()
        self.feature_extractor = TerrainFeatureExtractor()

        # Terrain classes
        self.classes = ['sand', 'rock', 'slope', 'hazard']
        self.class_to_idx = {cls: i for i, cls in enumerate(self.classes)}

        # Load existing model if available
        self._load_model()

    def train(self, training_images: List[np.ndarray], labels: List[str],
              test_size: float = 0.2, cv_folds: int = 5):
        """
        Train terrain classification model.

        Args:
            training_images: List of terrain images
            labels: Corresponding terrain labels
            test_size: Fraction of data for testing
            cv_folds: Number of cross-validation folds
        """
        print("🔬 Training terrain classification model...")

        # Extract features from all images
        features = []
        for img in training_images:
            feat = self.feature_extractor.extract_features(img)
            features.append(feat)

        X = np.array(features)
        y = np.array([self.class_to_idx[label] for label in labels])

        # Split data
        X_train, X_test, y_train, y_test = train_test_split(
            X, y, test_size=test_size, random_state=42, stratify=y
        )

        # Scale features
        X_train_scaled = self.scaler.fit_transform(X_train)
        X_test_scaled = self.scaler.transform(X_test)

        # Try multiple algorithms
        models = {
            'RandomForest': RandomForestClassifier(
                n_estimators=100,
                max_depth=10,
                random_state=42,
                class_weight='balanced'
            ),
            'SVM': SVC(
                kernel='rbf',
                C=1.0,
                gamma='scale',
                class_weight='balanced',
                random_state=42
            ),
            'GradientBoosting': GradientBoostingClassifier(
                n_estimators=100,
                learning_rate=0.1,
                max_depth=5,
                random_state=42
            )
        }

        best_model = None
        best_score = 0

        for name, model in models.items():
            print(f"  Evaluating {name}...")

            # Cross-validation
            cv_scores = cross_val_score(model, X_train_scaled, y_train, cv=cv_folds)
            mean_cv_score = np.mean(cv_scores)

            print(".3f")
            if mean_cv_score > best_score:
                best_score = mean_cv_score
                best_model = model
                self.model = model

        # Train best model on full training data
        if best_model:
            self.model.fit(X_train_scaled, y_train)

            # Evaluate on test set
            y_pred = self.model.predict(X_test_scaled)
            test_accuracy = accuracy_score(y_test, y_pred)

            print(".3f")
            print("\nClassification Report:")
            print(classification_report(y_test, y_pred, target_names=self.classes))

            # Save model
            self._save_model()

            return test_accuracy

        return 0.0

    def predict(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Predict terrain type for given image.

        Args:
            image: Terrain image to classify

        Returns:
            Dictionary with prediction results
        """
        if self.model is None:
            return {"error": "Model not trained"}

        # Extract features
        features = self.feature_extractor.extract_features(image)
        features_scaled = self.scaler.transform([features])

        # Predict
        prediction_idx = self.model.predict(features_scaled)[0]
        prediction_proba = self.model.predict_proba(features_scaled)[0]

        terrain_type = self.classes[prediction_idx]
        confidence = prediction_proba[prediction_idx]

        return {
            "terrain_type": terrain_type,
            "confidence": float(confidence),
            "all_probabilities": {
                cls: float(prob) for cls, prob in zip(self.classes, prediction_proba)
            }
        }

    def _save_model(self):
        """Save trained model to disk."""
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)

        model_data = {
            'model': self.model,
            'scaler': self.scaler,
            'classes': self.classes,
            'trained_at': time.time()
        }

        joblib.dump(model_data, self.model_path)
        print(f"💾 Model saved to {self.model_path}")

    def _load_model(self):
        """Load trained model from disk."""
        if os.path.exists(self.model_path):
            try:
                model_data = joblib.load(self.model_path)
                self.model = model_data['model']
                self.scaler = model_data['scaler']
                self.classes = model_data['classes']
                print(f"📂 Model loaded from {self.model_path}")
                print(f"   Trained: {time.ctime(model_data.get('trained_at', 0))}")
            except Exception as e:
                print(f"❌ Failed to load model: {e}")
                self.model = None
        else:
            print("ℹ️ No trained model found, will need training")
            self.model = None


class TerrainNeuralAnalyzer:
    """
    Advanced terrain analysis using TensorFlow/Keras.

    Uses deep learning for terrain classification with convolutional neural networks.
    Provides superior accuracy for complex terrain patterns.
    """

    def __init__(self, model_path: str = "models/terrain_cnn.h5"):
        if not TENSORFLOW_AVAILABLE:
            raise ImportError("TensorFlow required for neural terrain analysis")

        self.model_path = model_path
        self.model = None
        self.classes = ['sand', 'rock', 'slope', 'hazard']
        self.class_to_idx = {cls: i for i, cls in enumerate(self.classes)}

        # Image dimensions
        self.img_height = 64
        self.img_width = 64
        self.num_classes = len(self.classes)

        # Load existing model if available
        self._load_model()

    def build_model(self):
        """Build CNN model for terrain classification."""
        self.model = keras.Sequential([
            layers.Input(shape=(self.img_height, self.img_width, 3)),

            # Convolutional layers
            layers.Conv2D(32, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            layers.Conv2D(64, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            layers.Conv2D(128, (3, 3), activation='relu'),
            layers.MaxPooling2D((2, 2)),
            layers.Dropout(0.25),

            # Dense layers
            layers.Flatten(),
            layers.Dense(128, activation='relu'),
            layers.Dropout(0.5),
            layers.Dense(self.num_classes, activation='softmax')
        ])

        self.model.compile(
            optimizer='adam',
            loss='categorical_crossentropy',
            metrics=['accuracy']
        )

        return self.model

    def train(self, training_images: List[np.ndarray], labels: List[str],
              epochs: int = 50, batch_size: int = 32, validation_split: float = 0.2):
        """
        Train neural network for terrain classification.

        Args:
            training_images: List of terrain images
            labels: Corresponding terrain labels
            epochs: Number of training epochs
            batch_size: Training batch size
            validation_split: Validation data fraction
        """
        if self.model is None:
            self.build_model()

        print("🧠 Training neural terrain classifier...")

        # Preprocess images
        processed_images = []
        processed_labels = []

        for img, label in zip(training_images, labels):
            # Resize image
            resized = cv2.resize(img, (self.img_width, self.img_height)) if 'cv2' in globals() else img
            processed_images.append(resized)

            # Convert label to one-hot
            label_idx = self.class_to_idx[label]
            one_hot = np.zeros(self.num_classes)
            one_hot[label_idx] = 1
            processed_labels.append(one_hot)

        X = np.array(processed_images, dtype=np.float32) / 255.0  # Normalize
        y = np.array(processed_labels)

        # Train model
        history = self.model.fit(
            X, y,
            epochs=epochs,
            batch_size=batch_size,
            validation_split=validation_split,
            verbose=1
        )

        # Save model
        self._save_model()

        # Print results
        final_accuracy = history.history['accuracy'][-1]
        final_val_accuracy = history.history['val_accuracy'][-1]

        print(".3f")
        print(".3f")
        return final_val_accuracy

    def predict(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Predict terrain type using neural network.

        Args:
            image: Terrain image to classify

        Returns:
            Dictionary with prediction results
        """
        if self.model is None:
            return {"error": "Model not trained"}

        # Preprocess image
        resized = cv2.resize(image, (self.img_width, self.img_height)) if 'cv2' in globals() else image
        normalized = resized.astype(np.float32) / 255.0
        input_image = np.expand_dims(normalized, axis=0)

        # Predict
        predictions = self.model.predict(input_image, verbose=0)[0]

        # Get top prediction
        prediction_idx = np.argmax(predictions)
        terrain_type = self.classes[prediction_idx]
        confidence = predictions[prediction_idx]

        return {
            "terrain_type": terrain_type,
            "confidence": float(confidence),
            "all_probabilities": {
                cls: float(prob) for cls, prob in zip(self.classes, predictions)
            }
        }

    def _save_model(self):
        """Save trained neural network model."""
        os.makedirs(os.path.dirname(self.model_path), exist_ok=True)
        self.model.save(self.model_path)
        print(f"💾 Neural model saved to {self.model_path}")

    def _load_model(self):
        """Load trained neural network model."""
        if os.path.exists(self.model_path):
            try:
                self.model = keras.models.load_model(self.model_path)
                print(f"📂 Neural model loaded from {self.model_path}")
            except Exception as e:
                print(f"❌ Failed to load neural model: {e}")
                self.model = None
        else:
            print("ℹ️ No trained neural model found")
            self.model = None


class AdvancedTerrainAnalyzer:
    """
    Advanced terrain analysis combining ML and traditional CV approaches.

    Uses ensemble of ML models and computer vision for robust terrain classification.
    Provides confidence scores and handles uncertainty.
    """

    def __init__(self):
        self.ml_classifier = None
        self.neural_analyzer = None
        self.cv_fallback = None

        # Initialize components
        try:
            self.ml_classifier = TerrainMLClassifier()
        except ImportError:
            print("⚠️ scikit-learn not available, skipping ML classifier")

        try:
            self.neural_analyzer = TerrainNeuralAnalyzer()
        except ImportError:
            print("⚠️ TensorFlow not available, skipping neural analyzer")

        # CV fallback (from existing analyzer)
        try:
            from .terrain_analyzer import TerrainAnalyzer
            self.cv_fallback = TerrainAnalyzer()
        except ImportError:
            print("⚠️ CV fallback not available")

    def analyze_terrain(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Analyze terrain using best available method.

        Returns comprehensive analysis with confidence scores.
        """
        results = {}

        # Try neural network first (highest accuracy)
        if self.neural_analyzer:
            neural_result = self.neural_analyzer.predict(image)
            if "error" not in neural_result:
                results["neural_prediction"] = neural_result

        # Try ML classifier
        if self.ml_classifier:
            ml_result = self.ml_classifier.predict(image)
            if "error" not in ml_result:
                results["ml_prediction"] = ml_result

        # CV fallback
        if self.cv_fallback:
            try:
                cv_result = self.cv_fallback.classify_terrain(image)
                results["cv_prediction"] = self._convert_cv_to_dict(cv_result)
            except Exception as e:
                results["cv_error"] = str(e)

        # Ensemble decision
        final_prediction = self._ensemble_decision(results)

        return {
            "final_prediction": final_prediction,
            "all_methods": results,
            "analysis_timestamp": time.time()
        }

    def _convert_cv_to_dict(self, cv_classification: np.ndarray) -> Dict[str, Any]:
        """Convert CV classification array to dictionary format."""
        # Count pixels of each type
        unique, counts = np.unique(cv_classification, return_counts=True)
        pixel_counts = dict(zip(unique, counts))

        # Determine dominant terrain type
        dominant_class = unique[np.argmax(counts)]
        confidence = counts[np.argmax(counts)] / np.sum(counts)

        classes = ['sand', 'rock', 'slope', 'hazard']
        terrain_type = classes[dominant_class] if dominant_class < len(classes) else 'unknown'

        return {
            "terrain_type": terrain_type,
            "confidence": float(confidence),
            "pixel_distribution": {classes[i]: pixel_counts.get(i, 0) for i in range(len(classes))}
        }

    def _ensemble_decision(self, results: Dict[str, Any]) -> Dict[str, Any]:
        """Make ensemble decision from multiple analysis methods."""
        predictions = []

        # Collect predictions from all methods
        for method, result in results.items():
            if isinstance(result, dict) and "terrain_type" in result:
                predictions.append({
                    "method": method,
                    "terrain_type": result["terrain_type"],
                    "confidence": result["confidence"]
                })

        if not predictions:
            return {"error": "No valid predictions available"}

        # Simple voting with confidence weighting
        terrain_votes = {}
        total_confidence = 0

        for pred in predictions:
            terrain_type = pred["terrain_type"]
            confidence = pred["confidence"]

            if terrain_type not in terrain_votes:
                terrain_votes[terrain_type] = 0
            terrain_votes[terrain_type] += confidence
            total_confidence += confidence

        # Choose terrain type with highest weighted votes
        best_terrain = max(terrain_votes.items(), key=lambda x: x[1])
        ensemble_confidence = best_terrain[1] / total_confidence if total_confidence > 0 else 0

        return {
            "terrain_type": best_terrain[0],
            "confidence": float(ensemble_confidence),
            "methods_used": len(predictions),
            "ensemble_score": float(best_terrain[1])
        }


# Convenience functions
def create_terrain_analyzer():
    """Create advanced terrain analyzer instance."""
    return AdvancedTerrainAnalyzer()

def train_terrain_models(sample_images: List[np.ndarray], sample_labels: List[str]):
    """Train all available terrain analysis models."""
    analyzer = AdvancedTerrainAnalyzer()

    results = {}

    if analyzer.ml_classifier:
        print("🏃 Training ML classifier...")
        ml_accuracy = analyzer.ml_classifier.train(sample_images, sample_labels)
        results["ml_accuracy"] = ml_accuracy

    if analyzer.neural_analyzer:
        print("🧠 Training neural network...")
        nn_accuracy = analyzer.neural_analyzer.train(sample_images, sample_labels)
        results["neural_accuracy"] = nn_accuracy

    return results

# Export key components
__all__ = [
    'TerrainMLClassifier',
    'TerrainNeuralAnalyzer',
    'AdvancedTerrainAnalyzer',
    'create_terrain_analyzer',
    'train_terrain_models'
]
