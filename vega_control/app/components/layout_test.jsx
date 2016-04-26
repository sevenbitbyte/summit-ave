'use strict';

import React from 'react';
import AppBar from 'react-toolbox/lib/app_bar';
//import { AppBar, Checkbox, IconButton } from 'react-toolbox';
//import { Layout, NavDrawer, Panel, Sidebar } from 'react-toolbox';

class LayoutTest extends React.Component {


    constructor(props){
      super(props)

      this.state = {
          drawerActive: false,
          drawerPinned: false,
          sidebarPinned: false
      };

    }

    toggleDrawerActive () {
        this.setState({ drawerActive: !this.state.drawerActive });
    };

    toggleDrawerPinned () {
        this.setState({ drawerPinned: !this.state.drawerPinned });
    }

    toggleSidebar () {
        this.setState({ sidebarPinned: !this.state.sidebarPinned });
    };

    render() {
        return (
            <Layout>
                <NavDrawer active={this.state.drawerActive}
                    pinned={this.state.drawerPinned} permanentAt='xxxl'
                    onOverlayClick={ this.toggleDrawerActive.bind(this) }>
                    <p>
                        Navigation, account switcher, etc. go here.
                    </p>
                </NavDrawer>
                <Panel>
                    <AppBar><IconButton icon='menu' inverse={ true } onClick={ this.toggleDrawerActive.bind(this) }/></AppBar>
                    <div style={{ flex: 1, overflowY: 'auto', padding: '1.8rem' }}>
                        <h1>Main Content</h1>
                        <p>Main content goes here.</p>
                        <Checkbox label='Pin drawer' checked={this.state.drawerPinned} onChange={this.toggleDrawerPinned.bind(this)} />
                        <Checkbox label='Show sidebar' checked={this.state.sidebarPinned} onChange={this.toggleSidebar.bind(this)} />
                    </div>
                </Panel>
                <Sidebar pinned={ this.state.sidebarPinned } width={ 5 }>
                    <div><IconButton icon='close' onClick={ this.toggleSidebar.bind(this) }/></div>
                    <div style={{ flex: 1 }}>
                        <p>Supplemental content goes here.</p>
                    </div>
                </Sidebar>
            </Layout>
        );
    }
};

export default LayoutTest;
