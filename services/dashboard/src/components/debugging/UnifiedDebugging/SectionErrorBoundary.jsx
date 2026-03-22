import React from 'react';
import { AlertTriangle, RefreshCw } from 'lucide-react';

/**
 * Catches errors in one section so the rest of the dashboard remains usable (partial failure).
 */
export class SectionErrorBoundary extends React.Component {
  constructor(props) {
    super(props);
    this.state = { hasError: false, error: null };
  }

  static getDerivedStateFromError(error) {
    return { hasError: true, error };
  }

  componentDidCatch(error) {
    this.setState({ error });
  }

  render() {
    if (this.state.hasError) {
      return (
        <div
          className="rounded-lg border border-amber-500/30 bg-amber-500/10 p-4 max-w-2xl mx-auto mt-4"
          role="alert"
        >
          <div className="flex items-center gap-2 text-amber-400 font-medium">
            <AlertTriangle className="h-4 w-4 shrink-0" aria-hidden />
            Section unavailable
          </div>
          <p className="text-sm text-zinc-400 mt-1">
            This part of the dashboard failed to load. The rest of the interface is still available.
          </p>
          <button
            type="button"
            onClick={() => this.setState({ hasError: false, error: null })}
            className="mt-3 flex items-center gap-2 rounded border border-zinc-600 bg-zinc-800 px-3 py-1.5 text-sm text-zinc-200 hover:bg-zinc-700"
          >
            <RefreshCw className="h-3 w-3" />
            Retry
          </button>
        </div>
      );
    }
    return this.props.children;
  }
}
