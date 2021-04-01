import { isObjectLike } from "lodash";
const DELIMITER = " > ";

/**
 * Recursively walk the data object using a list of keys
 *
 * @param {object} data – object to be walked
 * @param {[string]} path – array of keys
 * @returns {object} - data point
 */
const getDataPoint = (data, path) => {
  if (Array.isArray(data)) {
    const key = path[0];
    path = path.slice(1);
    data = data.find(o => o.name === key);
  } else if (data.hasOwnProperty("value") && Array.isArray(data.value)) {
    const key = path[0];
    path = path.slice(1);
    data = data.value.find(o => o.name === key);
  } else if (isObjectLike(data)) {
    const isEnd = path.length === 0;
    if (isEnd) {
      return data;
    }
    const key = path[0];
    data = data[key];
    path = path.slice(1);
  } else {
    return undefined;
  }
  return getDataPoint(data, path);
};

export const getDataPointValue = (data, path) => {
  return getDataPoint(data, path).value;
};

/**
 * Recursively walk the data object to obtain a list of all data points' paths
 *
 * @param {object} data – object to be walked
 * @param {array} path – array of keys
 * @returns array of data point objects containing path and caption
 */
export const getAllPaths = (data, path = []) => {
  if (
    data.hasOwnProperty("crucial_data") &&
    data.hasOwnProperty("additional_data")
  ) {
    return getAllPaths(data.crucial_data, ["crucial_data"]).concat(
      getAllPaths(data.additional_data, ["additional_data"])
    );
  }
  return data
    .map(nestedData => {
      if (Array.isArray(nestedData.value)) {
        return getAllPaths(nestedData.value, [...path, nestedData.name]);
      }
      if (!isNaN(nestedData.value)) {
        const p = [...path, nestedData.name];
        return {
          caption: p.join(DELIMITER),
          path: p
        };
      }
      return undefined;
    })
    .flat();
};
