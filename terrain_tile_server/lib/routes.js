var Joi = require('joi');

var TerrainTileHandlers = require('./tile/handlers.js')

exports.routes = [];

exports.routes.push({
  method: 'GET',
  path: '/',
  config: {
    handler: function (request, reply) {
      return reply();
    }
  }
});

exports.routes.push({
  method: 'GET',
  path: '/terrain/tile/info/{lat},{long}.json',
  config: {
    validate : {
      params : {
        lat : Joi.number().min(-90).max(90).required(),
        long : Joi.number().min(-180).max(180).required(),
      }
    },
    handler: TerrainTileHandlers.getTileInfo
  }
});

exports.routes.push({
  method: 'GET',
  path: '/terrain/tile/elev/{lat},{long}.png',
  config: {
    validate : {
      params : {
        lat : Joi.number().min(-90).max(90).required(),
        long : Joi.number().min(-180).max(180).required(),
      },
      query : {
        scale : Joi.number().min(0.0).max(1.0).default(1.0)
      }
    },
    handler: TerrainTileHandlers.getTileElevPng
  }
});

exports.routes.push({
  method: 'GET',
  path: '/terrain/tile/data/{lat},{long}.json',
  config: {
    validate : {
      params : {
        lat : Joi.number().min(-90).max(90).required(),
        long : Joi.number().min(-180).max(180).required()
      }
    },
    handler: TerrainTileHandlers.getTileData
  }
});

exports.routes.push({
  method: 'POST',
  path : '/terrain/region/info.json',
  config : {
    validate : {
      payload : {
        bounds : Joi.array().length(2).items(
          Joi.object().keys({
          lat : Joi.number().min(-90).max(90).required(),
          long : Joi.number().min(-180).max(180).required()
        })
        )
      }
    },
    handler: TerrainTileHandlers.getRegionInfo
  }
});

exports.routes.push({
  method: 'POST',
  path : '/terrain/region/elev.png',
  config : {
    validate : {
      payload : {
        bounds : Joi.array().length(2).items(
          Joi.object().keys({
          lat : Joi.number().min(-90).max(90).required(),
          long : Joi.number().min(-180).max(180).required()
        })
        ),
        scale : Joi.number().min(0.0).max(1.0).default(1.0)
      }
    },
    handler: TerrainTileHandlers.getRegionElevPng
  }
});
