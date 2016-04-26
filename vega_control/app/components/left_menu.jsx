'use strict';

import React from 'react';
import Avatar from 'react-toolbox/lib/avatar';
import Button from 'react-toolbox/lib/button';
import Drawer from 'react-toolbox/lib/drawer';
import { List, ListItem, ListSubHeader, ListDivider, ListCheckbox } from 'react-toolbox/lib/list';
import { Layout, NavDrawer, Panel, Sidebar } from 'react-toolbox';

class DrawerTest extends React.Component {

  constructor(props){
    super(props)

    this.state = {
      active: false
    };

  }

  handleToggle () {
    //console.log(this)
    this.setState({active: !this.state.active});
  };

  render () {
    return (
      <div>
        <Button label='Show Drawer' raised accent onClick={this.handleToggle.bind(this)}/>
        <Drawer active={this.state.active} onOverlayClick={this.handleToggle.bind(this)}>
          <Avatar><img src="/images/HouseBot_Pic5.png"/></Avatar>
          Robot name
          <List selectable ripple>
            <ListSubHeader avatar='/images/HouseBot_Pic5.png' caption='Vega'>
              <Avatar><img src="/images/HouseBot_Pic5.png"/></Avatar>
            </ListSubHeader>
            <ListItem caption='Status' />
            <ListItem caption='Drive' />
            <ListItem caption='Settings' />
        </List>
        </Drawer>
      </div>
    );
  }
}

export default DrawerTest;
