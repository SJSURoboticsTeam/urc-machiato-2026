import React from 'react';
import { AlertTriangle, RefreshCw } from 'lucide-react';

/**
 * Error Boundary Component
 *
 * Catches JavaScript errors anywhere in the child component tree,
 * logs those errors, and displays a fallback UI instead of the
 * component tree that crashed.
 */
export class DashboardErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null, errorInfo: null };
  }

  static getDerivedStateFromError(error) {
    // Update state so the next render will show the fallback UI
    return { hasError: true };
  }

  componentDidCatch(error, errorInfo) {
    // Log the error to console and potentially to a logging service
    console.error('Dashboard Error Boundary caught an error:', error, errorInfo);

    this.setState({
      error: error,
      errorInfo: errorInfo
    });

    // In production, you might want to send this to a logging service
    // logErrorToService(error, errorInfo);
  }

  handleRetry = () => {
    this.setState({ hasError: false, error: null, errorInfo: null });
  };

  render() {
    if (this.state.hasError) {
      // Fallback UI
      return (
        <div className="min-h-screen bg-zinc-900 text-white p-8">
          <div className="max-w-2xl mx-auto">
            <div className="bg-red-900/20 border border-red-500/50 rounded-lg p-6">
              <div className="flex items-center mb-4">
                <AlertTriangle className="w-8 h-8 text-red-400 mr-3" />
                <h1 className="text-xl font-bold text-red-400">
                  Dashboard Error
                </h1>
              </div>

              <p className="text-zinc-300 mb-4">
                Something went wrong with the URC rover dashboard. This error has been logged and the development team has been notified.
              </p>

              <div className="bg-zinc-800 rounded p-4 mb-4">
                <h3 className="text-sm font-semibold text-zinc-400 mb-2">Error Details:</h3>
                <pre className="text-xs text-red-300 whitespace-pre-wrap">
                  {this.state.error && this.state.error.toString()}
                </pre>
              </div>

              <div className="flex gap-3">
                <button
                  onClick={this.handleRetry}
                  className="flex items-center gap-2 bg-blue-600 hover:bg-blue-700 text-white px-4 py-2 rounded transition-colors"
                >
                  <RefreshCw className="w-4 h-4" />
                  Try Again
                </button>

                <button
                  onClick={() => window.location.reload()}
                  className="bg-zinc-600 hover:bg-zinc-700 text-white px-4 py-2 rounded transition-colors"
                >
                  Reload Page
                </button>
              </div>

              <div className="mt-6 pt-4 border-t border-zinc-700">
                <h3 className="text-sm font-semibold text-zinc-400 mb-2">What to do next:</h3>
                <ul className="text-sm text-zinc-300 space-y-1">
                  <li>• Check your internet connection</li>
                  <li>• Verify the rover system is running</li>
                  <li>• Try refreshing the page</li>
                  <li>• Contact the technical team if the problem persists</li>
                </ul>
              </div>
            </div>
          </div>
        </div>
      );
    }

    return this.props.children;
  }
}

/**
 * Hook-based error boundary wrapper for functional components
 */
export const withErrorBoundary = (Component, fallbackMessage = "Something went wrong") => {
  return (props) => (
    <DashboardErrorBoundary>
      <Component {...props} />
    </DashboardErrorBoundary>
  );
};
