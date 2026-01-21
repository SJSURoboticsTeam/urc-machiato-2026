import React, { useState } from 'react';
import { SystemContextProvider, useSystemContext } from './context/SystemContext';
import { OfflineBanner } from './components/ui/OfflineBanner';
import { TopBar } from './components/TopBar';
import { TabNavigation } from './components/TabNavigation';
import { OverviewTab } from './components/tabs/OverviewTab';
import { MissionTab } from './components/tabs/MissionTab';
import { NetworkTab } from './components/tabs/NetworkTab';
import { TestingTab } from './components/tabs/TestingTab';
import { DebugTab } from './components/tabs/DebugTab';
import { AnalyticsTab } from './components/tabs/AnalyticsTab';
import { ConfigTab } from './components/tabs/ConfigTab';
import { MonitoringDashboard } from './components/testing-dashboard/MonitoringDashboard';

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
  const { isOnline } = useSystemContext();

  const renderTabContent = () => {
    switch (activeTab) {
      case 'overview':
        return <OverviewTab />;
      case 'mission':
        return <MissionTab />;
      case 'network':
        return <NetworkTab />;
      case 'monitoring':
        return <MonitoringDashboard />;
      case 'testing':
        return <TestingTab />;
      case 'debug':
        return <DebugTab />;
      case 'analytics':
        return <AnalyticsTab />;
      case 'config':
        return <ConfigTab />;
      default:
        return <OverviewTab />;
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
    </div>
  );
}

function App() {
  return (
    <SystemContextProvider>
      <AppContent />
    </SystemContextProvider>
  );
}

export default App;
