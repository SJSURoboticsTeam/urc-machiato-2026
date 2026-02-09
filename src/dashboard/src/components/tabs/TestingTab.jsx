import React, { useState } from 'react';
import { IntegratedTestingDashboard } from '../testing-dashboard/IntegratedTestingDashboard';
import { LegacyTestingView, TestingViewToggle } from './testing';

/**
 * TestingTab – view toggle between Integrated Dashboard and Component (Legacy) View.
 * Delegates to IntegratedTestingDashboard or LegacyTestingView.
 */
export const TestingTab = () => {
  const [viewMode, setViewMode] = useState('integrated');

  return (
    <div className="h-full flex flex-col">
      <TestingViewToggle viewMode={viewMode} setViewMode={setViewMode} />
      <div className="flex-1 overflow-hidden">
        {viewMode === 'integrated' ? (
          <IntegratedTestingDashboard />
        ) : (
          <div className="h-full overflow-y-auto">
            <LegacyTestingView />
          </div>
        )}
      </div>
    </div>
  );
};
