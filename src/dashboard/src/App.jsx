import React, { useState } from 'react';
import { AppProviders, useUIContext } from './context';
import { OfflineBanner } from './components/ui/OfflineBanner';
import { TopBar } from './components/TopBar';
import { TabNavigation } from './components/TabNavigation';
import { OverviewTab } from './components/tabs/OverviewTab';
import { MissionTab } from './components/tabs/MissionTab';
import { NetworkTab } from './components/tabs/NetworkTab';
import { TestingTab } from './components/tabs/TestingTab';
import { DebugTab } from './components/tabs/DebugTab';
import { DebuggingDashboard } from './components/debugging';
import { AnalyticsTab } from './components/tabs/AnalyticsTab';
import { ConfigTab } from './components/tabs/ConfigTab';
import { MonitoringDashboard } from './components/testing-dashboard/MonitoringDashboard';
import { SectionErrorBoundary } from './components/debugging/UnifiedDebugging/SectionErrorBoundary';

/**
 * Main App Component
 *
 * Clean, modular, context-aware interface for URC 2026 rover control system.
 * Features:
 * - Tab-based navigation
 * - Context-aware UI that adapts to system state
 * - Information-dense displays
 * - Progressive disclosure
 */
function AppContent() {
  const [activeTab, setActiveTab] = useState('overview');
  const { isOnline } = useUIContext();

  const renderTabContent = () => {
    switch (activeTab) {
      case 'overview':
        return (
          <SectionErrorBoundary>
            <OverviewTab />
          </SectionErrorBoundary>
        );
      case 'mission':
        return (
          <SectionErrorBoundary>
            <MissionTab />
          </SectionErrorBoundary>
        );
      case 'network':
        return (
          <SectionErrorBoundary>
            <NetworkTab />
          </SectionErrorBoundary>
        );
      case 'monitoring':
        return (
          <SectionErrorBoundary>
            <MonitoringDashboard />
          </SectionErrorBoundary>
        );
      case 'testing':
        return (
          <SectionErrorBoundary>
            <TestingTab />
          </SectionErrorBoundary>
        );
      case 'debug':
        return (
          <SectionErrorBoundary>
            <DebuggingDashboard />
          </SectionErrorBoundary>
        );
      case 'analytics':
        return (
          <SectionErrorBoundary>
            <AnalyticsTab />
          </SectionErrorBoundary>
        );
      case 'config':
        return (
          <SectionErrorBoundary>
            <ConfigTab />
          </SectionErrorBoundary>
        );
      default:
        return (
          <SectionErrorBoundary>
            <OverviewTab />
          </SectionErrorBoundary>
        );
    }
  };

  return (
    <div className="min-h-screen w-screen flex flex-col bg-zinc-950 text-zinc-100">
      {/* Top bar - always visible */}
      <TopBar />

      {/* Offline banner */}
      <OfflineBanner
        isOnline={isOnline}
        onRetry={() => window.location.reload()}
        lastSeen={null} // TODO: Add last connection timestamp
      />

      {/* Tab navigation */}
      <TabNavigation activeTab={activeTab} setActiveTab={setActiveTab} />

      {/* Tab content - scrollable */}
      <div className="flex-1 overflow-y-auto min-h-0">
        {renderTabContent()}
      </div>
    </div>
  );
}

function App() {
  return (
    <AppProviders>
      <AppContent />
    </AppProviders>
  );
}

export default App;
